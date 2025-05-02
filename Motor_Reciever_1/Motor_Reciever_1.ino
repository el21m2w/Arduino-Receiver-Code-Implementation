#include "CytronMotorDriver.h"

#define numMotors 3


CytronMD motor[numMotors] = {
  CytronMD(PWM_PWM, 8, 9),   // Motor 0
  CytronMD(PWM_PWM, 6, 7),   // Motor 1
  CytronMD(PWM_PWM, 4, 5)    // Motor 2
};

/*************************************************************
   Encoder pin arrays for full 4× decoding.
   On Arduino Mega, valid external interrupts:

*************************************************************/
const int EA[numMotors] = { 20, 18, 2 }; 
const int EB[numMotors] = { 21, 19, 3 };

/*************************************************************
   Limit switch pins
*************************************************************/
const int LS[numMotors] = { 42, 41, 40 };

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
const int numFrets = 15;

// Printing interval
unsigned long lastPrint = 0;
const unsigned long printInterval = 20;      // ms

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
  { 1, 18.36 },
  { 2, 52.41 },
  { 3, 82.41 },
  { 4, 113.18 },
  { 5, 144.04 },
  { 6, 169.66 },
  { 7, 195.59 },
  { 8, 218.32 },
  { 9, 242.45 },
  { 10, 264.64 },
  { 11, 286.51 },
  { 12, 305.18 },
  { 13, 323.53 },
  { 14, 338.92 },
  { 15, 357.59 }
  },
  { //Motor 2
  { 1, 16.4 },
  { 2, 49.68 },
  { 3, 82.17 },
  { 4, 111.62 },
  { 5, 143.41 },
  { 6, 169.06 },
  { 7, 216.76 },
  { 8, 216.76 },
  { 9, 239.41 },
  { 10, 261.2 },
  { 11, 283.15 },
  { 12, 302.91 },
  { 13, 319.24 },
  { 14, 338.22 },
  { 15, 355.95 }
  },
  { //Motor 3
  { 1, 18.12 },
  { 2, 54.44 },
  { 3, 84.75 },
  { 4, 117.24 },
  { 5, 148.33 },
  { 6, 173.80 },
  { 7, 201.76 },
  { 8, 225.27 },
  { 9, 248.31 },
  { 10, 270.65 },
  { 11, 292.52 },
  { 12, 313.50 },
  { 13, 328.84 },
  { 14, 347.36 },
  { 15, 363.68 }
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
  Serial2.begin(9600);

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
  if (Serial2.available() > 0) {
    String input = Serial2.readStringUntil('\n');
    Serial.println(input);
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
    for (int i = 0; i < 1; i++) {
      Serial.print(millis());
      Serial.print(" ");
      Serial.print(pos[2]);
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
    counter[0]++;
  } 
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    counter[0]--;
  }
  lastEncoded[0] = encoded;
}

void encoder1ISR() {
  byte MSB = digitalRead(EA[1]);
  byte LSB = digitalRead(EB[1]);
  byte encoded = (MSB << 1) | LSB;
  byte sum = (lastEncoded[1] << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    counter[1]++;
  } 
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    counter[1]--;
  }
  lastEncoded[1] = encoded;
}

void encoder2ISR() {
  byte MSB = digitalRead(EA[2]);
  byte LSB = digitalRead(EB[2]);
  byte encoded = (MSB << 1) | LSB;
  byte sum = (lastEncoded[2] << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    counter[2]++;
  } 
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    counter[2]--;
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
    if (Confirm_Correct[motorIndex] >= 100) { // confirm for stability
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
  Serial.println(String(firstChar) + String(secondChar));

  switch (firstChar_num) {
    case 1:
    case 2:
    case 3:
      // Activate that motor
      Active[firstChar_num - 1] = true;

      // Convert secondChar from hex if it's not 'T' or '0'
      if (secondChar != 'T' && secondChar != '0') {
        secondChar = hexToDecimal(secondChar);
      }

      // If secondChar == '0', do not move
      if (secondChar == '0' || secondChar == 0) {
        Serial.println("0 so don't move");
        Active[firstChar_num - 1] = false;
        motor[firstChar_num - 1].setSpeed(0);
        break; 
      }

      // Lookup fret distance
      FretDis = lookupFretLocations(secondChar, firstChar_num - 1);
      desiredPos[firstChar_num - 1] = FretDis;
      
      // Reset integral on new move
      totalError[firstChar_num - 1] = 0;

      Serial.print("Motor ");
      Serial.print(firstChar_num - 1);
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
