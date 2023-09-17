#include <Arduino.h>
#include "Servo.h"
#include "HX711_ADC.h"

const int RIGHT_MOTOR_OUT_PIN = A9;
const int LEFT_MOTOR_OUT_PIN = A8;
const int HX711_DOUT = A3;
const int HX711_SCK = A2;
const int RIGHT_POT_PIN = A1;
const int LEFT_POT_PIN = A0;

Servo rightMotor;
Servo leftMotor;
HX711_ADC LoadCell(HX711_DOUT, HX711_SCK); // HX711 constructor

// Constant
const float CALIBRATION_FACTOR = 430.56;
const int MOTOR_MAX_PULSE_WIDTH = 2000;
const int MOTOR_MIN_PULSE_WIDTH = 1000;
const int MAX_THROTTLE_PERCENTAGE = 100;
const int motorLowValue = MOTOR_MIN_PULSE_WIDTH;
const int motorHighValue = MOTOR_MIN_PULSE_WIDTH + 
                            (MOTOR_MAX_PULSE_WIDTH - MOTOR_MIN_PULSE_WIDTH) * MAX_THROTTLE_PERCENTAGE / 100;

// Variables
bool isStop = true;
int rightPotValue = motorLowValue;  // value from the right potentiometer
int leftPotValue = motorLowValue;  // value from the left potentiometer
const int serialPrintInterval = 0; // increase value to slow down serial print activity
unsigned long t = 0; // time from start the microcontroller

void initializeLoadcell();
void handleKeyPressEvent();
void printMenu();

void setup() {
  Serial.begin(9600);
  pinMode(RIGHT_POT_PIN, INPUT);
  pinMode(LEFT_POT_PIN, INPUT);
  analogReadResolution(12); // Only for microcontroller with 12bit ADC
  while (!Serial);

  // Initialize motor
  rightMotor.attach(RIGHT_MOTOR_OUT_PIN);
  rightMotor.writeMicroseconds(motorLowValue);

  leftMotor.attach(LEFT_MOTOR_OUT_PIN);
  leftMotor.writeMicroseconds(motorLowValue);

  // Initialize loadcell
  initializeLoadcell();

  printMenu();
  while (!Serial.read());
}

void loop() {
  if (!isStop) {
    // Change the speed of the motor with the potentiometer value
    rightPotValue = analogRead(RIGHT_POT_PIN);   // reads the value of the potentiometer (value between 0 and 4095)
    rightPotValue = map(rightPotValue, 0, 4095, motorLowValue, motorHighValue);
    rightMotor.writeMicroseconds(rightPotValue);    // Send the signal to the ESC

    leftPotValue = analogRead(LEFT_POT_PIN);   // reads the value of the potentiometer (value between 0 and 4095)
    leftPotValue = map(leftPotValue, 0, 4095, motorLowValue, motorHighValue);
    leftMotor.writeMicroseconds(leftPotValue);    // Send the signal to the ESC

    // Print value to Serial
    // Check for loadcell data
    static boolean newDataReady = 0;
    if (LoadCell.update()) newDataReady = true;
    if (newDataReady) {
      if (millis() > t + serialPrintInterval) {
        float i = LoadCell.getData();
        newDataReady = 0;
        t = millis();
        Serial.print("load_cell: ");
        Serial.print(i);
        Serial.print(", ");
        Serial.printf("right_esc: %d, left_esc: %d, time: %d\n", rightPotValue, leftPotValue, millis());
        // Serial.printf("\r\n");
      }
    }
  }

  handleKeyPressEvent();
}

// Function definitions
void initializeLoadcell() {
  LoadCell.begin();
  LoadCell.setReverseOutput(); // comment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; // set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(CALIBRATION_FACTOR);
  }
}

void handleKeyPressEvent() {
  if (Serial.available() > 0) {
    char inByte = Serial.read();

    // Motor calibration
    if (inByte == 'h') {
      rightMotor.writeMicroseconds(MIN_PULSE_WIDTH);
      leftMotor.writeMicroseconds(MIN_PULSE_WIDTH);
      rightMotor.detach();
      leftMotor.detach();
      Serial.println("Turn the power off");
      Serial.println("Turn the power on");
      Serial.println("When ok press 'h'");
      bool _resume = false;
      while (_resume == false) {
        if (Serial.available() > 0) {
          char tmp = Serial.read();

          if (tmp == 'h') {
            rightMotor.attach(RIGHT_MOTOR_OUT_PIN);

            rightMotor.writeMicroseconds(motorHighValue);
            delay(100);
            rightMotor.writeMicroseconds(motorLowValue);

            leftMotor.attach(LEFT_MOTOR_OUT_PIN);

            leftMotor.writeMicroseconds(motorHighValue);
            delay(100);
            leftMotor.writeMicroseconds(motorLowValue);

            _resume = true;
          }
        }
      }

      Serial.println("Done calibrate motors");
      Serial.print("High value: ");
      Serial.print(motorHighValue);
      Serial.print(", Low value: ");
      Serial.println(motorLowValue);
      printMenu();
      isStop = true;

      // Start/Stop
    } else if (inByte == 's') {
      isStop = !isStop;
      rightPotValue = motorLowValue;
      leftPotValue = motorLowValue;
      rightMotor.writeMicroseconds(rightPotValue);
      leftMotor.writeMicroseconds(leftPotValue);
      printMenu();

    // Tare load cell
    } else if (inByte == 't') {
      unsigned long stabilizingtime = 2000;
      LoadCell.start(stabilizingtime, true);
      LoadCell.setCalFactor(CALIBRATION_FACTOR);
    }
  }
}

void printMenu() {
  Serial.println();
  Serial.println("--- Menu ---");
  Serial.println("Press 'h' to calibrate two motors");
  Serial.println("Press 't' to tare the load cell again");
  Serial.println("Press 's' to start the program");
}