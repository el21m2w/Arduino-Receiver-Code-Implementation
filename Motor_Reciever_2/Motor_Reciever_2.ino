#include "CytronMotorDriver.h"

#define numMotors 3

/*************************************************************
   Cytron Motor Driver array 
   If your Cytron library doesn't support PWM_PWM mode,
   use PWM_DIR (and wire accordingly).
*************************************************************/
CytronMD motor[numMotors] = {
  CytronMD(PWM_PWM, 4, 5),   // Motor 0
  CytronMD(PWM_PWM, 6, 7),   // Motor 1
  CytronMD(PWM_PWM, 8, 9)    // Motor 2
};

/*************************************************************
   Encoder pin arrays for full 4× decoding.
   On Arduino Mega, valid external interrupts:
     2,3,18,19,20,21
*************************************************************/
const int EA[numMotors] = { 2, 18, 20 }; 
const int EB[numMotors] = { 3, 19, 21 };

/*************************************************************
   Limit switch pins
*************************************************************/
const int LS[numMotors] = { 40, 41, 42 };

/*************************************************************
   Encoder, positioning, and PID variables
*************************************************************/
volatile long counter[numMotors] = { 0 };  // volatile for ISR
double dpr[numMotors] = { 0.078110302, 0.078110302, 0.078110302 }; // mm per encoder tick
double pos[numMotors] = { 0 };
double desiredPos[numMotors] = { 0 };
bool Active[numMotors] = { false };
int Confirm_Correct[numMotors] = { 0 };

// PID parameters
double kp[numMotors] = { 5, 5, 5 };
double ki[numMotors] = { 0, 0, 0 };
double kd[numMotors] = { 0.03531, 0.03531, 0.03531 };

// PID state
double prevError[numMotors] = { 0, 0, 0 };
double totalError[numMotors] = { 0, 0, 0 };
unsigned long lastUpdate[numMotors] = { 0, 0, 0 };
double dt[numMotors];
double error[numMotors];
double errorRate[numMotors] = { 0 };
unsigned long errorRateTime[numMotors] = { 0 };
const unsigned long errorRateInterval = 100;  // ms

// Printing interval
unsigned long lastPrint = 0;
const unsigned long printInterval = 1000;      // ms
const int numFrets = 15;

/*************************************************************
   This array stores previous states for each encoder so we
   can do 4× decoding in the ISRs
*************************************************************/
volatile byte lastEncoded[numMotors] = { 0, 0, 0 };

/*************************************************************
   FretLocations struct and map
*************************************************************/
struct FretLocations {
  char key;
  double value;
};

FretLocations FretLocationMap[numMotors][numFrets] = {
  { //Motor 1
  { 1, 0 },
  { 2, 32.57 },
  { 3, 64.99 },
  { 4, 96.39},
  { 5, 125.91},
  { 6, 152.91},
  { 7, 178.33},
  { 8, 203.71},
  { 9, 226.13},
  { 10, 248.31},
  { 11, 267.84},
  { 12, 286.27},
  { 13, 304.71},
  { 14, 323.91},
  { 15, 340.48}
  },
  { //Motor 2
  { 1, 3.05},
  { 2, 36.95 },
  { 3, 70.06 },
  { 4, 99.12 },
  { 5, 128.1},
  { 6, 154.97 },
  { 7, 181.22},
  { 8, 204.88},
  { 9, 227.14},
  { 10, 251.12},
  { 11, 271.12},
  { 12, 290.02},
  { 13, 309.39},
  { 14, 326.81},
  { 15, 345.64}
  },
  { //Motor 3
  { 1, 0 },
  { 2, 34.99 },
  { 3, 67.64 },
  { 4, 97.09 },
  { 5, 125.6 },
  { 6, 152.71 },
  { 7, 176.29 },
  { 8, 202.15 },
  { 9, 224.33 },
  { 10, 246.28 },
  { 11, 267.61 },
  { 12, 285.96 },
  { 13, 304.01 },
  { 14, 322.05 },
  { 15, 338.84 }
  }
};

/*************************************************************
   Function Declarations
*************************************************************/
void encoder0ISR();
void encoder1ISR();
void encoder2ISR();

void initialisePosition(CytronMD& motor, int motorIndex, int limitSwitch);
void updateMotorPID(int motorIndex, unsigned long now);
int handleCharInput(char firstChar, char secondChar);
int hexToDecimal(char hex);
double lookupFretLocations(char key, int motorIndex);

/*************************************************************
   Setup
*************************************************************/
void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);

  // Configure encoder pins as inputs with pull-ups
  for (int i = 0; i < numMotors; i++) {
    pinMode(EA[i], INPUT_PULLUP);
    pinMode(EB[i], INPUT_PULLUP);
  }

  // Configure limit switch pins
  for (int i = 0; i < numMotors; i++) {
    pinMode(LS[i], INPUT_PULLUP);
    Serial.println("LS PULLED UP");
  }

  // Stop all motors initially
  for (int i = 0; i < numMotors; i++) {
    motor[i].setSpeed(0);
    lastUpdate[i] = millis();
  }

  // Attach interrupts for full 4× decoding
  attachInterrupt(digitalPinToInterrupt(EA[0]), encoder0ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EB[0]), encoder0ISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(EA[1]), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EB[1]), encoder1ISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(EA[2]), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EB[2]), encoder2ISR, CHANGE);

  // Initialize each motor's home position
  for (int i = 0; i < numMotors; i++) {
    initialisePosition(motor[i], i, LS[i]);
    delay(500);
  }

  Serial.println("Initialized");
}

/*************************************************************
   Main Loop
*************************************************************/
void loop() {
  // Handle serial input for commands
  if (Serial3.available() > 0) {
    String input = Serial3.readStringUntil('\n');
      char firstChar = input.charAt(0);
      char secondChar = input.charAt(1);
      handleCharInput(firstChar, secondChar);
  }

  unsigned long now = millis();

  // Update positions and run PID if motor is active
  for (int i = 0; i < numMotors; i++) {
    if (Active[i]) {
      // Read the volatile encoder count safely
      noInterrupts();
      pos[i] = counter[i] * dpr[i];
      interrupts();

      updateMotorPID(i, now);
    }
  }

  // Print status periodically
  /*if (now - lastPrint >= printInterval) {
    lastPrint = now;
    for (int i = 0; i < numMotors; i++) {
      Serial.print("Motor");
      Serial.print(i);
      Serial.print(" Pos: ");
      Serial.print(pos[i]);
      Serial.print(", Target: ");
      Serial.print(desiredPos[i]);
      Serial.print(", Error: ");
      Serial.print(error[i]);
      if (i < numMotors - 1) {
        Serial.print(" | ");
      }
    }
    Serial.println();
  }*/
}

/*************************************************************
   Full 4× Decoding ISRs
   Each motor's A&B lines trigger the same ISR
   The logic is:
     1) Read A and B => combine into "encoded"
     2) Combine with lastEncoded => see how we moved
     3) ++ or -- the counter
     4) Save the new encoded
*************************************************************/
void encoder0ISR() {
  // Read both channels for motor 0
  byte MSB = digitalRead(EA[0]);
  byte LSB = digitalRead(EB[0]);
  byte encoded = (MSB << 1) | LSB;  // 2-bit value: [A,B]
  byte sum = (lastEncoded[0] << 2) | encoded;

  // These bit patterns indicate clockwise vs counterclockwise
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    counter[0]--;
  } 
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    counter[0]++;
  }
  lastEncoded[0] = encoded;
}

void encoder1ISR() {
  byte MSB = digitalRead(EA[1]);
  byte LSB = digitalRead(EB[1]);
  byte encoded = (MSB << 1) | LSB;
  byte sum = (lastEncoded[1] << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    counter[1]--;
  } 
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    counter[1]++;
  }
  lastEncoded[1] = encoded;
}

void encoder2ISR() {
  byte MSB = digitalRead(EA[2]);
  byte LSB = digitalRead(EB[2]);
  byte encoded = (MSB << 1) | LSB;
  byte sum = (lastEncoded[2] << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    counter[2]--;
  } 
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    counter[2]++;
  }
  lastEncoded[2] = encoded;
}

/*************************************************************
   Homing / Initialization
*************************************************************/
void initialisePosition(CytronMD& thisMotor, int motorIndex, int limitSwitch) {
  Serial.print("Initializing motor ");
  Serial.println(motorIndex);

  // Move slowly in negative direction until limit switch reads HIGH
  // (Assuming 'LOW' means pressed; adjust if it's reversed)
  thisMotor.setSpeed(-50);
  while (digitalRead(limitSwitch) == LOW) {
    delay(10); 
  }

  // Stop motor and reset encoder
  thisMotor.setSpeed(0);
  noInterrupts();
  counter[motorIndex] = 0;
  lastEncoded[motorIndex] = 0; // reset last encoded state
  interrupts();

  // Reset PID variables
  pos[motorIndex] = 0;
  desiredPos[motorIndex] = 0;
  prevError[motorIndex] = 0;
  totalError[motorIndex] = 0;
  errorRate[motorIndex] = 0;
  errorRateTime[motorIndex] = millis();

  Serial.print("Motor ");
  Serial.print(motorIndex);
  Serial.println(" initialized");
}

/*************************************************************
   PID Control
*************************************************************/
void updateMotorPID(int motorIndex, unsigned long now) {
  // Time since last update
  dt[motorIndex] = (double)(now - lastUpdate[motorIndex]) / 1000.0;
  lastUpdate[motorIndex] = now;

  // Compute current error
  error[motorIndex] = desiredPos[motorIndex] - pos[motorIndex];

  // Integral windup guard
  if (fabs(error[motorIndex]) < 1.0) {
    // Near target, clamp integral
    totalError[motorIndex] = constrain(totalError[motorIndex], -10, 10);
  } else if (error[motorIndex] * prevError[motorIndex] < 0) {
    // Error changed sign => reset integral
    totalError[motorIndex] = 0;
  } else {
    totalError[motorIndex] += error[motorIndex] * dt[motorIndex];
    totalError[motorIndex] = constrain(totalError[motorIndex], -50, 50);
  }

  // Derivative at slower update intervals to reduce noise
  if (now - errorRateTime[motorIndex] >= errorRateInterval) {
    double errorRateDt = (double)(now - errorRateTime[motorIndex]) / 1000.0;
    errorRate[motorIndex] = (error[motorIndex] - prevError[motorIndex]) / errorRateDt;
    prevError[motorIndex] = error[motorIndex];
    errorRateTime[motorIndex] = now;
  }

  // Are we there yet
  double tolerance = 2.0;  // mm
  if (fabs(error[motorIndex]) < tolerance) {
    Confirm_Correct[motorIndex]++;
    if (Confirm_Correct[motorIndex] >= 1000) { // confirm for stability
      Active[motorIndex] = false;
      motor[motorIndex].setSpeed(0);
      Serial.print("Motor ");
      Serial.print(motorIndex);
      Serial.println(" reached target position");

      return;
    }
  } else {
    Confirm_Correct[motorIndex] = 0;
  }

  // Calculate PID output
  double output = (kp[motorIndex] * error[motorIndex]) 
                + (ki[motorIndex] * totalError[motorIndex])
                + (kd[motorIndex] * errorRate[motorIndex]);

  // Optional dead zone for small errors
  if (fabs(error[motorIndex]) < 0.5) {
    output = 0;
  }

  // Constrain output and set motor speed
  output = constrain(output, -200, 200);
  motor[motorIndex].setSpeed(output);
}

/*************************************************************
   Serial Input Handler
*************************************************************/
int handleCharInput(char firstChar, char secondChar) {
  int firstChar_num = hexToDecimal(firstChar);
  double FretDis = 0;

  switch (firstChar_num) {
    case 4:
    case 5:
    case 6:
      // Activate that motor
      Active[firstChar_num - 4] = true;

      // Convert secondChar from hex if it's not 'T' or '0'
      if (secondChar != 'T' && secondChar != '0') {
        secondChar = hexToDecimal(secondChar);
      }

      // If secondChar == '0', do not move
      if (secondChar == '0' || secondChar == 0) {
        Serial.println("0 so don't move");
        Active[firstChar_num - 4] = false;
        motor[firstChar_num - 4].setSpeed(0);
        break; 
      }

      // Lookup fret distance
      FretDis = lookupFretLocations(secondChar, (firstChar_num - 4));
      desiredPos[firstChar_num - 4] = FretDis;
      
      // Reset integral on new move
      totalError[firstChar_num - 4] = 0;

      Serial.print("Motor ");
      Serial.print(firstChar_num - 4);
      Serial.print(" moving to position ");
      Serial.println(FretDis);
      break;

    default:
      Serial.println("Input is outside the expected range");
      break;
  }

  return 0;
}

/*************************************************************
   Hex -> Decimal Utility
*************************************************************/
int hexToDecimal(char hex) {
  if (hex >= '0' && hex <= '9') {
    return hex - '0';
  } else if (hex >= 'A' && hex <= 'F') {
    return hex - 'A' + 10;
  } else {
    return -1;
  }
}

/*************************************************************
   Fret Lookup
*************************************************************/
double lookupFretLocations(char key, int motorIndex) {
  // If 'T', read a custom distance from Serial
  if (key == 'T') {
    while (true) {
      if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        int customDist = input.toInt();
        return customDist;
      }
    }
  }

  // Normal dictionary lookup
  for (int i = 0; i < numFrets; i++) {
    if (FretLocationMap[motorIndex][i].key == key) {
      return FretLocationMap[motorIndex][i].value;
    }
  }

  // Not found
  return -1;
}
