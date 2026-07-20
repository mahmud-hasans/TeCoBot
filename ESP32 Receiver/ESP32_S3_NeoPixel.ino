// ESP32 3S Robot 1 Code 5-22-24


/*
Written by Sydney Spiegel 
Modified by Mahmud Hasan Saikot
Output Shaft Style D-shaft 
Motor Type Brushed DC 
Output Shaft Support Brass Case 
Gear Material Metal 
Weight  0.42 oz (12g) 
Voltage (Nominal) 12V 
Voltage Range (Recommended) 6V - 12V 
Speed (No Load @ 12VDC) 90 rpm 
Current (No Load @ 12VDC) 70mA 
Current (Stall @ 12VDC) 1600mA 
Torque (Stall @ 12VDC)  70 oz-in 
Gearbox Style Straight Cut Spur 
Connector Type  ZH Series JST 6-pin connector (1.5mm Pitch) 
Encoder: Cycles Per Revolution (Motor Shaft)  3 
Encoder: Countable Events Per Revolution (Output Shaft) 3,575.0855 
Gear Ratio  297.924:1 
Encoder Type  Relative, Quadrature 
Encoder Sensor Type Magnetic (Hall Effect) 
*/

#include <esp_now.h>  
#include <WiFi.h>  
#include <ESP32Encoder.h> 
#include <Adafruit_NeoPixel.h>

#define PIN 38        // ESP32-S3 built-in RGB LED
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


bool newCommandReceived = false; 

// Encoder objects  
ESP32Encoder encoder1;  
ESP32Encoder encoder2;  
ESP32Encoder encoder3;  

#define DELAYVAL 1000 // Delay for one second

const int freq = 1000; 
const int resolution = 8; 

// TB6612FNG Motor Driver 1
// Motor 1: Brushed DC motor with Quad Encoder - Pulls cable 
#define PWM_1A_PIN 4  // white 
#define DIR1_A1_PIN 5 // blue
#define DIR1_A2_PIN 6  // green
int PWM_Motor1_Speed = 0;
const int motor1_Channel = 1; 
// Encoder pins Motor 1 
#define ENC1A 1 // blue  
#define ENC1B 2 // purple/green
float motor1_Setpoint = 0; // Setpoint in encoder pulses
float motor1_Current_Position = 0; // Current positon in encoder pulses
float motor1_Rotations = 0;  // Variable to count rotations
bool motor_1_Moving = false; 

// Motor 2: Brushed DC motor with Quad Encoder - Pulls cable  
#define PWM_1B_PIN 7   // white  
#define DIR1_B1_PIN 15 // blue 
#define DIR1_B2_PIN 16 // green 
int PWM_Motor2_Speed = 0;  
const int motor2_Channel = 2;  
// Encoder pins Motor 2  
#define ENC2A 37 // blue  
#define ENC2B 36 // purple/green 
float motor2_Setpoint = 0;  // Setpoint in encoder pulses
float motor2_Current_Position = 0; // Current positon in encoder pulses
float motor2_Rotations = 0;  // Variable to count rotations 
bool motor_2_Moving = false;

// TB6612FNG Driver 2 Controls motor 3 and 4  
// Motor 3: Brushed DC motor with Quad Encoder - Pulls cable  
#define PWM_2A_PIN 17 // white  
#define DIR2_A1_PIN 18 // blue 
#define DIR2_A2_PIN 8 // green 
int PWM_Motor3_Speed = 0;  
const int motor3_Channel = 3;  
// Encoder pins Motor 3  
#define ENC3A 47 // blue  
#define ENC3B 21 // purple
float motor3_Setpoint = 0;  // Setpoint in encoder pulses
float motor3_Current_Position = 0; // Current positon in encoder pulses
float motor3_Rotations = 0;  // Variable to count rotations
bool motor_3_Moving = false; 

// Motor 4: Brushed DC Motor without encoder - Opens/Closes Upper Claw  
#define PWM_2B_PIN 3 // white  
#define DIR2_B1_PIN 9 // blue 
#define DIR2_B2_PIN 10 // green 
int PWM_Motor4_Speed = 255;  
const int motor4_Channel = 4;  
//float motor4_Setpoint = 0; 
//float motor4_Current_Position = 0; 

// TB6612FNG Driver Controls motor 5  
// Motor 5: Brushed DC Motor without encoder - Opens/Closes Lower Claw  
#define PWM_3A_PIN 12 // white  
#define DIR3_A1_PIN 13 // blue 
#define DIR3_A2_PIN 14 // green 
int PWM_Motor5_Speed = 255;  
const int motor5_Channel = 5; 
//float motor4_Setpoint = 0; 
//float motor4_Current_Position = 0;  

// Encoder counts per revolution, gear ratios, etc., as needed  
const int CPR = 12; // 12 counts per revolution of small shaft on back of motor 
float gearRatio = 297.924; // Gear ratio for motors = 298:1  
float encoderCountsPerRevolution = 3575; //gearRatio*CPR ~ 3,575 
float tolerance = 100; // Adjust this if needed - if current position is within this value from setpoint motor stops

float Revolutions = 1; // Start at 1 revolutions 
float RevolutionsShort = 0.25; 
float RevolutionsMedium = 1;
float RevolutionsLong = 2;
float RevolutionsHuge = 4; 
float RevolutionsInchworm = 5;
float MaxRevolutions = 8;
float MinRevolutions = -9;
float InchRevolutions = 4;

// Rotation Variables
int revolutionCounter = 0;
int revolutionIterations = 0; 

// For Upper Claw  
unsigned long upperClawStartTime = 0; // To store the start time of motor movement  
unsigned long upperClawRunTime = 250; // Motor run time in milliseconds (0.25 seconds)  
bool upperClawRunning = false; // To track whether the motor is currently running  

// For Lower Claw  
unsigned long lowerClawStartTime = 0; // To store the start time of motor movement  
unsigned long lowerClawRunTime = 250; // Motor run time in milliseconds (0.25 seconds)  
bool lowerClawRunning = false; // To track whether the motor is currently running 

// Movement Gaits Variables
bool rolling_Left = false; 
bool rolling_Right = false; 
bool inchworming_Forward = false; 
bool turning_Left = false; 
bool turning_Right = false;
bool pipeclimbing_Up = false; 
bool pipeclimbing_Down = false; 

bool setUpComplete = false;

int moveInchwormState = 0;// variable to keep track of inchworm state

//Structure example to receive data  
//Must match the sender structure  
typedef struct robotCommands_struct {  
bool connected; // true or false 
int robotNumber; // 1,2,3,4 ect...
int manualInput; // for manual control 
int move; // pre programed movements 360 bend, roll, pipe climb ect..
int iterations; // how many times to perform a gait sequence.
int gait; // perform gait sequence for set number of iterations 
} robotCommands_struct;  

//Create a struct_message  
robotCommands_struct myCommand;  

//callback function that will be executed when data is received 
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) { 
  memcpy(&myCommand, incomingData, sizeof(myCommand));
  
  Serial.println("New command received:"); 
  Serial.println("Connected?: " + String(myCommand.connected)); 
  Serial.println("Robot Number: " + String(myCommand.robotNumber)); 
  Serial.println("Last Manual Input: " + String(myCommand.manualInput)); 
  Serial.println("Last Move Command: " + String(myCommand.move)); 
  Serial.println("Last Set Iterations: " + String(myCommand.iterations)); 
  Serial.println("Last Gait Command: " + String(myCommand.gait)); 

  newCommandReceived = true; // Set flag to true to indicate new data has been received 
  
  if (myCommand.connected){
  interpretManualCommands();
  }
  
  
  
} 

void upperClawTimer(){ 
  // Timer for upper claw motor  
  if (upperClawRunning && (millis() - upperClawStartTime >= upperClawRunTime || myCommand.manualInput == 60 )) {  
    ledcWrite(motor4_Channel, 0); // Stop the motor by setting PWM to 0 
    upperClawRunning = false; // Indicate that the motor has stopped  
  // Optional: Reset manualInput to prevent repeated starts  
  //myCommand.manualInput = 0;  
  }  
} 

 
void lowerClawTimer(){ 
  // Timer for upper claw motor  
  if (lowerClawRunning && (millis() - lowerClawStartTime >= lowerClawRunTime || myCommand.manualInput == 60)) {  
  // Stop the motor after running for the specified time  
  ledcWrite(motor5_Channel, 0); // Stop the motor by setting PWM to 0  
  lowerClawRunning = false; // Indicate that the motor has stopped  
  // Optional: Reset manualInput to prevent repeated starts  
  //myCommand.manualInput = 0;  
  }  
} 

void interpretManualCommands() {  

  switch (myCommand.manualInput) {  

  case 1: // Upper Claw Close 
  if (!upperClawRunning){ 
  // Set Direction Pins 
  digitalWrite(DIR2_B1_PIN, HIGH);  
  digitalWrite(DIR2_B2_PIN, LOW);  
  // Drive Motor 4 
  ledcWrite(motor4_Channel, PWM_Motor4_Speed);  
  // Set timer and flag 
  upperClawStartTime = millis(); // Save the start time  
  upperClawRunning = true; // Indicate that the motor is running  
  } 
  break;  

  case 2: // Upper Claw Open 
  if (!upperClawRunning){ 
  // Set Direction Pins 
  digitalWrite(DIR2_B1_PIN, LOW);  
  digitalWrite(DIR2_B2_PIN, HIGH);  
  // Drive Motor 4 
  ledcWrite(motor4_Channel, PWM_Motor4_Speed);  
  // Set timer and flag 
  upperClawStartTime = millis(); // Save the start time  
  upperClawRunning = true; // Indicate that the motor is running  
  } 
  break;  

  case 3: // Lower Claw Close 
  if (!lowerClawRunning){ 
  // Set Direction Pins 
  digitalWrite(DIR3_A1_PIN, HIGH);  
  digitalWrite(DIR3_A2_PIN, LOW);  
  // Drive Motor 5 
  ledcWrite(motor5_Channel, PWM_Motor5_Speed);  
  // Set timer and flag 
  lowerClawStartTime = millis(); // Save the start time  
  lowerClawRunning = true; // Indicate that the motor is running  
  } 
  break;  

  case 4: // Lower Claw Open 
  if (!lowerClawRunning){ 
  // Set Direction Pins 
  digitalWrite(DIR3_A1_PIN, LOW);  
  digitalWrite(DIR3_A2_PIN, HIGH);  
  // Drive Motor 5 
  ledcWrite(motor5_Channel, PWM_Motor5_Speed);  
  // Set timer and flag 
  lowerClawStartTime = millis(); // Save the start time  
  lowerClawRunning = true; // Indicate that the motor is running
  } 
  break;  

  case 5: // Move Motor 1 forward  
  if (motor1_Rotations + Revolutions <= MaxRevolutions) {  
  motor1_Setpoint += encoderCountsPerRevolution * Revolutions;  
  motor1_Rotations += Revolutions;  
  } else {  
  Serial.println("Motor 1 Forward Limit Reached!");  
  }  
  break;  

  case 6: // Move Motor 1 backward  
  if (motor1_Rotations - Revolutions >= MinRevolutions) {  
  motor1_Setpoint -= encoderCountsPerRevolution * Revolutions;  
  motor1_Rotations -= Revolutions;  
  } else {  
  Serial.println("Motor 1 Backward Limit Reached!");  
  }  
  break;  

  case 7: // Move Motor 2 forward  
  if (motor2_Rotations + Revolutions <= MaxRevolutions) {  
  motor2_Setpoint += encoderCountsPerRevolution * Revolutions;  
  motor2_Rotations += Revolutions;  
  } else {  
  Serial.println("Motor 2 Forward Limit Reached!");  
  }  
  break;  

  case 8: // Move Motor 2 backward  
  if (motor2_Rotations - Revolutions >= MinRevolutions) {  
  motor2_Setpoint -= encoderCountsPerRevolution * Revolutions;  
  motor2_Rotations -= Revolutions;  
  } else {  
  Serial.println("Motor 2 Backward Limit Reached!");  
  }  
  break;  

  case 9: // Move Motor 3 forward  
  if (motor3_Rotations + Revolutions <= MaxRevolutions) {  
  motor3_Setpoint += encoderCountsPerRevolution * Revolutions;  
  motor3_Rotations += Revolutions;  
  } else {  
  Serial.println("Motor 3 Forward Limit Reached!");  
  }  
  break;  

  case 10: // Move Motor 3 backward  
  if (motor3_Rotations - Revolutions >= MinRevolutions) {  
  motor3_Setpoint -= encoderCountsPerRevolution * Revolutions;  
  motor3_Rotations -= Revolutions;  
  } else {  
  Serial.println("Motor 3 Backward Limit Reached!");  
  }  
  break;  

  // Adjusting revolution increments  
  case 11:  
  Revolutions = RevolutionsShort;  
  upperClawRunTime = 250;
  lowerClawRunTime = 250;
  break;  

  case 12:  
  Revolutions = RevolutionsMedium;  
  upperClawRunTime = 500;
  lowerClawRunTime = 500;
  break;  

  case 13:  
  Revolutions = RevolutionsLong;
  // default = 1000
  upperClawRunTime = 1000;
  lowerClawRunTime = 1000;  
  break;  

  case 14: // Collapse robot to MaxRevolutions  
  // Check each motor to ensure we don't exceed MaxRevolutions  
  if (motor1_Rotations < MaxRevolutions) {  
  motor1_Setpoint += (MaxRevolutions - motor1_Rotations) * encoderCountsPerRevolution;  
  motor1_Rotations = MaxRevolutions;  
  }  
  if (motor2_Rotations < MaxRevolutions) {  
  motor2_Setpoint += (MaxRevolutions - motor2_Rotations) * encoderCountsPerRevolution;  
  motor2_Rotations = MaxRevolutions;  
  }  
  if (motor3_Rotations < MaxRevolutions) {  
  motor3_Setpoint += (MaxRevolutions - motor3_Rotations) * encoderCountsPerRevolution;  
  motor3_Rotations = MaxRevolutions;  
  }  
  Serial.println("Fully Collapse");  
  break;  

  case 15: // Zero the robot's position  
  motor1_Setpoint = motor2_Setpoint = motor3_Setpoint = 0;  
  motor1_Rotations = motor2_Rotations = motor3_Rotations = 0;  
  Serial.println("Robot reset to zero position.");  
  break;  

   case 16: // Adjust Command
  Revolutions = 0.1;  
  Serial.println("Robot minor adjust mode.");  
  break;  

  case 17: // STOP Command
  myCommand.move = 0;
  myCommand.manualInput = 0;
  moveInchwormState = 0;
  revolutionCounter = 0;
  Revolutions = RevolutionsMedium;  
  upperClawRunTime = 500;
  lowerClawRunTime = 500;

  break; 

  case 18:  
  Revolutions = RevolutionsHuge;
  upperClawRunTime = 2000;
  lowerClawRunTime = 2000;  
  break;  

  case 19:  
  Revolutions = RevolutionsLong;
  upperClawRunTime = 10000;
  lowerClawRunTime = 7000;  
  break;  

  case 20: // STOP Command
  myCommand.manualInput = 60;
  break;
  
  case 21: // Switch Command
  myCommand.manualInput = 98;
  openTopClaw();
  closeBottomClaw();
  break;

  case 22:
  myCommand.manualInput = 99;
  closeTopClaw();
  openBottomClaw();
  break;

  default:  
  // Handle any unrecognized commands  
  Serial.print("Unrecognized command received: ");  
  Serial.println(myCommand.manualInput);  
  break;  
  }  

}  

void motor_control(int motor_channel, float motor_Current_Position, float motor_Setpoint) {
    // Calculate the error between the desired position and the current position
    float error = motor_Setpoint - motor_Current_Position;

    float high_Speed = 255;
    float medium_Speed = 200;
    float low_Speed = 100; 

    // If the error is within the tolerance, stop the motor
    if (fabs(error) <= tolerance) {
        ledcWrite(motor_channel, 0);// stop motor
        switch (motor_channel) {
        case motor1_Channel:
            motor_1_Moving = false;
            break;
        case motor2_Channel:
            motor_2_Moving = false;
            break;
        case motor3_Channel:
            motor_3_Moving = false;
            break;
      }
      return;
    }

    // Determine the direction pins for the given motor channel
    int dir_pin1, dir_pin2;
    switch (motor_channel) {
        case motor1_Channel:
            dir_pin1 = DIR1_A1_PIN;
            dir_pin2 = DIR1_A2_PIN;
            motor_1_Moving = true;
            break;
        case motor2_Channel:
            dir_pin1 = DIR1_B1_PIN;
            dir_pin2 = DIR1_B2_PIN;
            motor_2_Moving = true;
            break;
        case motor3_Channel:
            dir_pin1 = DIR2_A1_PIN;
            dir_pin2 = DIR2_A2_PIN;
            motor_3_Moving = true;
            break;
        default:
            Serial.print("Invalid motor channel: ");
            Serial.println(motor_channel);
            return;
    }

    // Set the direction of the motor based on the sign of the error
    digitalWrite(dir_pin1, error > 0 ? HIGH : LOW);
    digitalWrite(dir_pin2, error > 0 ? LOW : HIGH);

    // Determine the motor speed
    // Try and make the motor slightly overshoot when pulling becuase the motor needs to pull hard to collapse then if it overshoots it can slowly release
    // cable until the error between the setpoint and current position is relatively small
    // If the error is greater than rotation (1/2),(1/3) ect., set speed to maximum if not map error to set speed between low and medium speed
    
      // Set motor speed based on the direction and magnitude of the error
    int speed;
    if (error > 0) {
        speed = high_Speed; // if error is pos, move motor forward and pull cable at maximum speed
    } else {
        if (fabs(error) > encoderCountsPerRevolution / 2) {// else if error negative and greater than 1/2 revolution from setpoint
            speed = high_Speed; // Release cable at high speed
        } else {
            speed = map(fabs(error), 0, (encoderCountsPerRevolution/2), low_Speed, medium_Speed);
        }
    }
    speed = constrain(speed,low_Speed, high_Speed); // Ensure the speed is within the bounds

    // Set the motor speed
    ledcWrite(motor_channel, speed);

    // Print the current error only when the motor is moving
    if (speed > low_Speed) {
        Serial.print("Motor ");
        Serial.print(motor_channel);
        Serial.print(" error: ");
        Serial.println(error);
        Serial.println(speed);
    }
}

// void status_Light() {
//     // Check if the device is connected
//     if (myCommand.connected) { 
//         // Initial assumption is that the robot is idle and connected (green light)
//         neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0); // Green
//         // Check if any motor is running or if the claws are operating, then set to red
//         if (upperClawRunning || lowerClawRunning || motor_1_Moving || motor_2_Moving || motor_3_Moving) {
//             neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0); // red
//         }
//     } else {
//         // If not connected, set the light color to white
//         neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,RGB_BRIGHTNESS); // white
//     }
// }

// new LED code to fix issues across all
void status_Light() {
    if (myCommand.connected) { 
        // Default: connected + idle → green
        pixels.setPixelColor(0, pixels.Color(0, 255, 0));
        
        // Any motion → red
        if (upperClawRunning || lowerClawRunning ||
            motor_1_Moving || motor_2_Moving || motor_3_Moving) {
            pixels.setPixelColor(0, pixels.Color(255, 0, 0));
        }
    } else {
        // Not connected → white
        pixels.setPixelColor(0, pixels.Color(255, 255, 255));
    }

    pixels.show();
}


void moveRotation() {
  if (myCommand.connected && (myCommand.move == 1 || myCommand.move == 2)) {
    switch (myCommand.move) {
      case 1: // rotate robot CW
        if (revolutionCounter == 0) {
          // Step 1: Move motor 2 forward
          motor2_Setpoint += encoderCountsPerRevolution * Revolutions;
          motor2_Rotations += Revolutions;
          revolutionCounter = 1;
        } else if (revolutionCounter == 1) {
          // Step 2: Move motor 3 forward
          if (fabs(motor2_Current_Position - motor2_Setpoint) < tolerance) {
            motor3_Setpoint += encoderCountsPerRevolution * Revolutions;
            motor3_Rotations += Revolutions;
            revolutionCounter = 2;
          }
        } else if (revolutionCounter == 2) {
          // Step 3: Move motor 2 backward
          if (fabs(motor3_Current_Position - motor3_Setpoint) < tolerance) {
            motor2_Setpoint -= encoderCountsPerRevolution * Revolutions;
            motor2_Rotations -= Revolutions;
            revolutionCounter = 3;
          }
        } else if (revolutionCounter == 3) {
          // Step 4: Move motor 1 forward
          if (fabs(motor2_Current_Position - motor2_Setpoint) < tolerance) {
            motor1_Setpoint += encoderCountsPerRevolution * Revolutions;
            motor1_Rotations += Revolutions;
            revolutionCounter = 4;
          }
        } else if (revolutionCounter == 4) {
          // Step 5: Move motor 3 backward
          if (fabs(motor1_Current_Position - motor1_Setpoint) < tolerance) {
            motor3_Setpoint -= encoderCountsPerRevolution * Revolutions;
            motor3_Rotations -= Revolutions;
            revolutionCounter = 5;
          }
        } else if (revolutionCounter == 5) {
          // Step 6: Move motor 2 forward
          if (fabs(motor3_Current_Position - motor3_Setpoint) < tolerance) {
            motor2_Setpoint += encoderCountsPerRevolution * Revolutions;
            motor2_Rotations += Revolutions;
            revolutionCounter = 6;
            Serial.println("6");
          }
        } else if (revolutionCounter == 6) {
          // Step 7: Move motor 1 backward
          if (fabs(motor2_Current_Position - motor2_Setpoint) < tolerance) {
            motor1_Setpoint -= encoderCountsPerRevolution * Revolutions;
            motor1_Rotations -= Revolutions;
            revolutionCounter = 7;
            Serial.println("7");
          }
        } else if (revolutionCounter == 7) {
          // Check for iteration completion
            Serial.println("else if 7 ");
            if (fabs(motor1_Current_Position - motor1_Setpoint) < tolerance) {
            revolutionIterations++;
            revolutionCounter = 1;
            Serial.println("7 if 1");

            } else {
              // End the movement sequence
              Serial.println("else");

            }
          }
        break; // case:1 break

      case 2: // rotate robot CCW
        if (revolutionCounter == 0) {
        // Step 1: Move motor 2 forward
        motor2_Setpoint += encoderCountsPerRevolution * Revolutions;
        motor2_Rotations += Revolutions;
        revolutionCounter = 1;
        } else if (revolutionCounter == 1) {
          // Step 2: Move motor 3 forward
          if (fabs(motor2_Current_Position - motor2_Setpoint) < tolerance) {
            motor1_Setpoint += encoderCountsPerRevolution * Revolutions;
            motor1_Rotations += Revolutions;
            revolutionCounter = 2;
          }
        } else if (revolutionCounter == 2) {
          // Step 3: Move motor 2 backward
          if (fabs(motor1_Current_Position - motor1_Setpoint) < tolerance) {
            motor2_Setpoint -= encoderCountsPerRevolution * Revolutions;
            motor2_Rotations -= Revolutions;
            revolutionCounter = 3;
          }
        } else if (revolutionCounter == 3) {
          // Step 4: Move motor 1 forward
          if (fabs(motor2_Current_Position - motor2_Setpoint) < tolerance) {
            motor3_Setpoint += encoderCountsPerRevolution * Revolutions;
            motor3_Rotations += Revolutions;
            revolutionCounter = 4;
          }
        } else if (revolutionCounter == 4) {
          // Step 5: Move motor 3 backward
          if (fabs(motor3_Current_Position - motor3_Setpoint) < tolerance) {
            motor1_Setpoint -= encoderCountsPerRevolution * Revolutions;
            motor1_Rotations -= Revolutions;
            revolutionCounter = 5;
          }
        } else if (revolutionCounter == 5) {
          // Step 6: Move motor 2 forward
          if (fabs(motor1_Current_Position - motor1_Setpoint) < tolerance) {
            motor2_Setpoint += encoderCountsPerRevolution * Revolutions;
            motor2_Rotations += Revolutions;
            revolutionCounter = 6;
            Serial.println("6");
          }
        } else if (revolutionCounter == 6) {
          // Step 7: Move motor 1 backward
          if (fabs(motor2_Current_Position - motor2_Setpoint) < tolerance) {
            motor3_Setpoint -= encoderCountsPerRevolution * Revolutions;
            motor3_Rotations -= Revolutions;
            revolutionCounter = 7;
            Serial.println("7");
          }
        } else if (revolutionCounter == 7) {
          // Check for iteration completion
            Serial.println("else if 7 ");
            if (fabs(motor3_Current_Position - motor3_Setpoint) < tolerance) {
            revolutionIterations++;
            revolutionCounter = 1;
            Serial.println("7 if 1");

            } else {
              // End the movement sequence
              Serial.println("else");

            }
          }
        break; // case:2 break
    }
  }
}

void openTopClaw() {
    if (!upperClawRunning) {  
        upperClawRunTime = 10000;
        Serial.print("Opening");
        // Set Direction Pins to open the upper claw
        digitalWrite(DIR2_B1_PIN, LOW);
        digitalWrite(DIR2_B2_PIN, HIGH);

        // Drive Motor 4 to open the upper claw
        ledcWrite(motor4_Channel, PWM_Motor4_Speed);

        // Set timer and flag
        upperClawStartTime = millis(); // Save the start time
        upperClawRunning = true; // Indicate that the motor is running
    } else {
        if (millis() - upperClawStartTime >= upperClawRunTime) {
          // Stop the motor and reset the flag
          upperClawRunning = false; // set claw running flag back to false
        }
      }
}

void closeTopClaw() {
    if (!upperClawRunning) {
        // Set the timer for the upper claw
        upperClawRunTime = 10000;

        // Set Direction Pins to close the upper claw
        digitalWrite(DIR2_B1_PIN, HIGH);
        digitalWrite(DIR2_B2_PIN, LOW);

        // Drive Motor 4 to close the upper claw
        ledcWrite(motor4_Channel, PWM_Motor4_Speed);

        // Set timer and flag
        upperClawStartTime = millis(); // Save the start time
        upperClawRunning = true; // Indicate that the motor is running
    } else {
        if (millis() - upperClawStartTime >= upperClawRunTime) {
          // Stop the motor and reset the flag
          upperClawRunning = false; // set claw running flag back to false
        }
      }
}

void openBottomClaw() {
    if (!lowerClawRunning) {
        // Set the timer for the lower claw
        lowerClawRunTime = 10000;

        // Set Direction Pins to open the lower claw
        digitalWrite(DIR3_A1_PIN, LOW);
        digitalWrite(DIR3_A2_PIN, HIGH);

        // Drive Motor 5 to open the lower claw
        ledcWrite(motor5_Channel, PWM_Motor5_Speed);

        // Set timer and flag
        lowerClawStartTime = millis(); // Save the start time
        lowerClawRunning = true; // Indicate that the motor is running
    } else {
        if (millis() - upperClawStartTime >= upperClawRunTime) {
          // Stop the motor and reset the flag
          lowerClawRunning = false; // set claw running flag back to false
        }
      }
}

void closeBottomClaw() {
    if (!lowerClawRunning) {
        // Set the timer for the lower claw
        lowerClawRunTime = 10000;

        // Set Direction Pins to close the lower claw
        digitalWrite(DIR3_A1_PIN, HIGH);
        digitalWrite(DIR3_A2_PIN, LOW);

        // Drive Motor 5 to close the lower claw
        ledcWrite(motor5_Channel, PWM_Motor5_Speed);

        // Set timer and flag
        lowerClawStartTime = millis(); // Save the start time
        lowerClawRunning = true; // Indicate that the motor is running
    } else {
        if (millis() - upperClawStartTime >= upperClawRunTime) {
          // Stop the motor and reset the flag
          lowerClawRunning = false; // set claw running flag back to false
        }
      }
}

void moveInchworm() {
  if (myCommand.connected && (myCommand.move == 3)) {
    // robot is odd number 
    //!!!ENSURE ROBOT HAS BOTH CLAWS CLOSED AND MOTOR 1 IS UP!!!!
    //!!! ROBOT WILL NOT STOP MOVING UNTIL DISCONNECTED OR myCommand.move not equal to 3 !!!
    if (myCommand.robotNumber == 1 || myCommand.robotNumber == 4) {
      if (moveInchwormState == 0) {
        Revolutions = RevolutionsInchworm;
        // wait for both claws to stop moving
        if (!upperClawRunning && !lowerClawRunning) {
          openBottomClaw();
          moveInchwormState = 1;    // state 1: front claw opened, robot extended
        }
      } else if (moveInchwormState == 1) {
        // wait till upper claw has stopped
        if (!upperClawRunning && !lowerClawRunning) {
          motor1_Setpoint += encoderCountsPerRevolution * Revolutions;
          motor1_Rotations += Revolutions;
          //pull motor 3 cable 4 revolutions
          motor3_Setpoint += encoderCountsPerRevolution * Revolutions;
          motor3_Rotations += Revolutions;
          moveInchwormState = 2;  // state 2: front claw opened, robot arched (motor 2 up/un-used)
          Revolutions = RevolutionsInchworm-3;
        }
      } else if (moveInchwormState == 2) {
        // wait for motor 1 and 3
        if ((fabs(motor1_Current_Position - motor1_Setpoint) < tolerance) && (fabs(motor3_Current_Position - motor3_Setpoint) < tolerance)) {
          closeBottomClaw();
          openTopClaw();
          moveInchwormState = 3;  // state 3: robot arched, front claw closed, bottom claw opened
          Serial.println("State 3");
        }
      } else if(moveInchwormState == 3){
        if (!upperClawRunning && !lowerClawRunning){
          // make sure claws are done opening/closing
          Serial.println("Time to Relax!");
          motor1_Setpoint -= encoderCountsPerRevolution * Revolutions;
          motor1_Rotations -= Revolutions;
          //pull motor 3 cable 4 revolutions
          motor3_Setpoint -= encoderCountsPerRevolution * Revolutions;
          motor3_Rotations -= Revolutions;
          moveInchwormState = 4;  // state 4: robot extended, front claw closed, back claw opened
          }
      } else if (moveInchwormState == 4) {
        // wait for motor 1 and 3 to reach setpoints
        if ((fabs(motor1_Current_Position - motor1_Setpoint) < tolerance) && (fabs(motor3_Current_Position - motor3_Setpoint) < tolerance)) {
          openBottomClaw();
          closeTopClaw();
          Serial.println("State 4");
          moveInchwormState = 1;  // state 5: robot extened, front claw open, back claw closed
        }
      }
    } // end of if myCommand.robotNumber == odd

    // robot is even number
    else if (myCommand.robotNumber == 2 || myCommand.robotNumber == 4) {
      // Add logic for even-numbered robots if needed.
    }
  }
}



void setup() {

  Serial.begin(115200); 

  pixels.begin();
  pixels.clear();
  pixels.show();

  memset(&myCommand, 0, sizeof(myCommand)); // Ensure all fields are zero

  //Set device as a Wi-Fi Station  
  WiFi.mode(WIFI_STA);  

  //Init ESP-NOW   
  if (esp_now_init() != ESP_OK) {  
  Serial.println("Error initializing ESP-NOW");  
  return;  
  }  
  // Once ESPNow is successfully Init, we will register for recv CB to  
  // get recv packer info  
  esp_now_register_recv_cb(OnDataRecv);

  // Motor driver 1 Motor 1  
  pinMode(DIR1_A1_PIN, OUTPUT);  
  pinMode(DIR1_A2_PIN, OUTPUT);  
  pinMode(PWM_1A_PIN, OUTPUT);  
  // Motor 1 PWM Setup 
  ledcSetup(motor1_Channel, freq, resolution); // Channel 0  
  ledcAttachPin(PWM_1A_PIN, motor1_Channel);  
  // Encoder Pins for Motor 1 
  pinMode(ENC1A, INPUT);  
  pinMode(ENC1B, INPUT);  
  encoder1.attachHalfQuad(ENC1A, ENC1B);  

  // Motor driver 1 Motor 2  
  pinMode(DIR1_B1_PIN, OUTPUT);  
  pinMode(DIR1_B2_PIN, OUTPUT);  
  pinMode(PWM_1B_PIN, OUTPUT);  
  // Motor 2 PWM Setup 
  ledcSetup(motor2_Channel, freq, resolution);  
  ledcAttachPin(PWM_1B_PIN, motor2_Channel);  
  // Encoder Pins for Motor 2  
  pinMode(ENC2A, INPUT);  
  pinMode(ENC2B, INPUT); 
  encoder2.attachHalfQuad(ENC2A, ENC2B);  

  // Motor driver 2 Motor 3  
  pinMode(DIR2_A1_PIN, OUTPUT);  
  pinMode(DIR2_A2_PIN, OUTPUT);  
  pinMode(PWM_2A_PIN, OUTPUT);  
  // Motor 3 PWM Setup 
  ledcSetup(motor3_Channel, freq, resolution);   
  ledcAttachPin(PWM_2A_PIN, motor3_Channel);  
  // Encoder Pins for Motor 3 
  pinMode(ENC3A, INPUT);  
  pinMode(ENC3B, INPUT);  
  encoder3.attachHalfQuad(ENC3A, ENC3B);  

  // Motor driver 2 Motor 4  
  pinMode(DIR2_B1_PIN, OUTPUT);  
  pinMode(DIR2_B2_PIN, OUTPUT);  
  pinMode(PWM_2B_PIN, OUTPUT);  
  // Motor 4 PWM Setup 
  ledcSetup(motor4_Channel, freq, resolution); 
  ledcAttachPin(PWM_2B_PIN, motor4_Channel);  

  // Motor driver 3 Motor 5  
  pinMode(DIR3_A1_PIN, OUTPUT);  
  pinMode(DIR3_A2_PIN, OUTPUT);  
  pinMode(PWM_3A_PIN, OUTPUT);  
  // Motor 5 PWM Setup 
  ledcSetup(motor5_Channel, freq, resolution); 
  ledcAttachPin(PWM_3A_PIN, motor5_Channel);  

  // set starting count value after attaching 
  encoder1.setCount(0); 
  encoder1.clearCount(); 
  encoder2.setCount(0); 
  encoder2.clearCount(); 
  encoder3.setCount(0); 
  encoder3.clearCount(); 

  // Initial state  
  motor1_Current_Position = encoder1.getCount();  
  motor2_Current_Position = encoder2.getCount();  
  motor3_Current_Position = encoder3.getCount(); 
  
  Serial.print("Getting Ready...");
  Serial.print("Ready in 3...");
  neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0); // Green
  delay(DELAYVAL); 
  Serial.print("Ready in 2...");
  neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,0); // Yellow
  delay(DELAYVAL);
  Serial.print("Ready in 1...");
  neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,RGB_BRIGHTNESS); // white
  delay(DELAYVAL);
  Serial.print("Ready");

  myCommand.iterations = 5;

  setUpComplete = true;
}

void loop() {

  status_Light();

  // Update the current motor position based on the encoder count
  motor1_Current_Position = encoder1.getCount() * 2; 
  motor2_Current_Position = encoder2.getCount() * 2;
  motor3_Current_Position = encoder3.getCount() * 2; 

  motor_control(motor1_Channel, motor1_Current_Position, motor1_Setpoint);
  motor_control(motor2_Channel, motor2_Current_Position, motor2_Setpoint);
  motor_control(motor3_Channel, motor3_Current_Position, motor3_Setpoint);
  
  // Turn of Claw Motors if needed
  upperClawTimer();
  lowerClawTimer();

  moveRotation(); // rotation gait if myCommand.move == 1 or 2 
  moveInchworm(); // Inchworm gait if myCommand.move == 3 or 4
  // Serial.println(myCommand.connected);
  // Serial.println(myCommand.manualInput);
  // Serial.println(myCommand.move);

  // A short delay to prevent flooding the serial output too quickly
  delay(100); // Adjust delay as needed to balance responsiveness with readability
}
