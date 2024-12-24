#include <Servo.h>
#include <Keypad.h>
#include "HX711.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#define ENC_A 3
#define ENC_B 2

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 

// Global Variables
Servo 
myServo;
HX711 scale;

int servoPin = 46; // Servo connected to pin 9

int buttonGroundFloor = 4; // Ground Floor Button
int buttonFirstFloor = 38;  // First Floor Button
int buttonSecondFloor = 39; // Second Floor Button
const byte ROWS = 1; // Number of rows on the keypad
const byte COLS = 3; // Number of columns on the keypad
//motor 1 inputs
const int motorPin1 = 24;//52 // Motor control pin 1 (to L293D Input 1)
const int motorPin2 = 28; //50 // Motor control pin 2 (to L293D Input 2)
//motor 2 inputs
const int motorPin3 = 9;
const int motorPin4 = 10;
//motor 3 inputs
const int motorPin5 = 48;
const int motorPin6 = 50;
//FOR FLOOR 1
const int limitSwitch_1 = 6; // Start switch connected to pin 2
const int limitSwitch_2 = 49;  // Stop switch connected to pin 3
//FOR FLOOR 2
const int limitSwitch_3 = 51; // Start switch connected to pin 2
const int limitSwitch_4 = 53;  // Stop switch connected to pin 3
//FOR FLOOR 3
const int limitSwitch_5 = 45; // Start switch connected to pin 2
const int limitSwitch_6 = 43;  // Stop switch connected to pin 3

bool motorRunning = true;
float floor_0=0;
// Pin definitions
const int LOADCELL_DOUT_PIN = 5; // Data pin (DT)
const int LOADCELL_SCK_PIN = 7;  // Clock pin (SCK)

// Define rotary encoder pins

unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 10;
int Angle = 0;

volatile int counter = 0;
int i = 90;

// Initialize HX711


char keys[ROWS][COLS] = {
  {'1', '2', '3'}
};
byte rowPins[ROWS] = {8}; // Connect rows to these Arduino pins
byte colPins[COLS] = {11, 12, 13}; // Connect columns to these Arduino pins

int currentState = 1; // Elevator starts at Ground Floor
int servoPosition = 90; // Servo's default position (neutral)
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  
   //Attach interrupts
  attachInterrupt(digitalPinToInterrupt(switch1Pin), enterInterrupt, RISING);  // When switch 1 is pressed
  attachInterrupt(digitalPinToInterrupt(switch2Pin), exitInterrupt, RISING);   // When switch 2 is pressed
  
  // Initialize Servo
  myServo.attach(servoPin);
  myServo.write(servoPosition);

  // Initialize Buttons
  pinMode(buttonGroundFloor, INPUT_PULLUP);
  pinMode(buttonFirstFloor, INPUT_PULLUP);
  pinMode(buttonSecondFloor, INPUT_PULLUP);
    // Set motor pins as output
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
   // motor pins as output
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);   
     // motor pins as output
  pinMode(motorPin5, OUTPUT);
  pinMode(motorPin6, OUTPUT);  


  // Set switch pins as input with pull-up resistors
  pinMode(limitSwitch_1, INPUT);
  pinMode(limitSwitch_2, INPUT);
    // Set switch pins as input with pull-up resistors
  pinMode(limitSwitch_3, INPUT);
  pinMode(limitSwitch_4, INPUT);
    // Set switch pins as input with pull-up resistors
  pinMode(limitSwitch_5, INPUT);
  pinMode(limitSwitch_6, INPUT);
  
  
  Serial.println("HX711 Weight Measurement");

  // Initialize the scale
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Calibration factor (adjust based on your calibration)
  scale.set_scale(2280.f); // Set the calibration factor
  scale.tare();            // Reset the scale to 0
  

  // Initialize Serial Monitor for Debugging
  Serial.begin(9600);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);
  lcd.begin(16,2);//Defining 16 columns and 2 rows of lcd display
  lcd.backlight();//To Power ON the back light
}

void loop() {
  // Read Button States
  bool groundFloorPressed = digitalRead(buttonGroundFloor);
  bool firstFloorPressed = digitalRead(buttonFirstFloor);
  bool secondFloorPressed = digitalRead(buttonSecondFloor);
  char key = keypad.getKey();
  //loadcell();

  if (interruptActive) {
    // When interrupt is active, perform some action
    Serial.println("Interrupt is active!");
    delay(1000);  // Simulating some work while interrupt is active
  } else {
    // Idle state, no interrupt is active
    Serial.println("Interrupt is not active.");
    delay(1000);
  }
  // Elevator Logic
  switch (currentState) {
    case 1: // Ground Floor
      if (groundFloorPressed || key == '1') {
        Serial.println("Ground Floor Button Pressed. Holding Position.");
        //Clean the screen
         lcd.setCursor(0,1); //Defining positon to write from first column,first row .
         lcd.print("Ground_Floor"); //You can write 16 Characters per line .
        //myServo.write(90); // Stay in position
        startMotor_1();
          // Check if the start switch is pressed
        

      }
      else if (firstFloorPressed || key == '2') {
        loadcell();
        Serial.println("Moving to First Floor.");
        startrevMotor_1();
        
         moveServo1_2(1);
           lcd.clear();//Clean the screen
           lcd.setCursor(0,0); //Defining positon to write from first column,first row .
           lcd.print("First_Floor"); //You can write 16 Characters per line .

        startMotor_2();
         // Return to neutral
        currentState = 2;
      }
      else if (secondFloorPressed || key == '3') {
        loadcell();
        startrevMotor_1();
        moveServo1_3(1);
          lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("Second_Floor"); //You can write 16 Characters per line .
        startMotor_3();
        currentState = 3;
      }
      break;

    case 2: // First Floor
      if (firstFloorPressed || key == '2') {
        Serial.println("First Floor Button Pressed. Holding Position.");
          lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("First_Floor");
        //myServo.write(90); // Stay in position
        startMotor_2();
      }
      else if (groundFloorPressed || key == '1') {
        loadcell();
        startrevMotor_2();
        moveServo2_1(1);
          lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("Ground_Floor");
        startMotor_1();
        currentState = 1;
      }
      else if (secondFloorPressed || key == '3') {
        loadcell();
        startrevMotor_2();
        moveServo2_3(1);
          lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("Second_Floor");
        startMotor_3();
        currentState = 3;
      }
      break;

    case 3: // Second Floor
      if (secondFloorPressed || key == '3') {
        Serial.println("Second Floor Button Pressed. Holding Position.");
          lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("Second_Floor");
        //myServo.write(90); // Stay in position
        startMotor_3();
      }
      else if (groundFloorPressed || key == '1') {
        loadcell();
        startrevMotor_3();
        moveServo3_1(1);
                 lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("Ground_Floor");
        startMotor_1();
        currentState = 1;
      }
      else if (firstFloorPressed || key == '2') {
        loadcell();
        startrevMotor_3();
        moveServo3_2(1);
                 lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("First_Floor");
        startMotor_2();
        currentState = 2;
      }
      break;

    default:
      Serial.println("Error: Unknown State.");
      currentState = 1; // Reset to Ground Floor
      break;
  }
}

// Helper Function to Move Servo
void moveServo1_2(int position) {
         lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("Elevate");
          Serial.println("Elevate000");

   for(int i=90;i<150;i++){
    
     myServo.write(i);
          delay(50);
          Serial.println( i);
   }

          
  while(counter>-32){
    myServo.write(150);
    Serial.println(counter);
  }
      for(int i=150;i>90&&counter>-43;i--){
          myServo.write(i);
          delay(50);
      }
  myServo.write(90);
}

void moveServo1_3(int position) {
   lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("Elevate");

   for(int i=90;i<150;i++){
     myServo.write(i);
          delay(50);
          Serial.println( i);
   }

          
  while(counter>-74){
    myServo.write(150);
    Serial.println(counter);
  }
for(int i=150;i>90&&counter>-86;i--){
          myServo.write(i);
          delay(50);
        }
  myServo.write(90);
}
void moveServo2_3(int position) {
   lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("Elevate");

   for(int i=90;i<150;i++){
     myServo.write(i);
          delay(50);
          Serial.println( i);
   }

          
  while(counter>-74){
    myServo.write(150);
    Serial.println(counter);
  }
for(int i=150;i>90&&counter>-86;i--){
          myServo.write(i);
          delay(50);
        }
  myServo.write(90);
}

void moveServo2_1(int position) {
   lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("Down");

   for(int i=90;i>20;i--){
          myServo.write(i);
          delay(50);
        }

          
  while(counter<-31){
    myServo.write(20);
    Serial.println(counter);
  }
for(int i=20;i<90&&counter<0;i++){
          myServo.write(i);
          delay(50);
        }
  myServo.write(90);
}
void moveServo3_2(int position) {
   lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("Down");

   for(int i=90;i>20;i--){
          myServo.write(i);
          delay(50);
        }

          
  while(counter<-71){
    myServo.write(20);
    Serial.println(counter);
  }
for(int i=20;i<90&&counter<-47;i++){
          myServo.write(i);
          delay(50);
        }
  myServo.write(90);
}
void moveServo3_1(int position) {
          lcd.clear();//Clean the screen
          lcd.setCursor(0,0); //Defining positon to write from first column,first row .
          lcd.print("Down");

   for(int i=90;i>20;i--){
          myServo.write(i);
          delay(50);
        }

          
  while(counter<-24){
    myServo.write(20);
    Serial.println(counter);
  }
for(int i=20;i<90&&counter<0;i++){
          myServo.write(i);
          delay(50);
        }
  myServo.write(90);
}

void startMotor_1() {
  // Rotate motor in one direction
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  while (digitalRead(limitSwitch_1) == HIGH) {
     Serial.println(digitalRead(limitSwitch_1));
    // Wait until the limit switch is triggered
  }

  // Once the limit switch is triggered, stop the motor
  stopMotor_1();

  Serial.println("Motor stopped by limit switch.");
}

void startrevMotor_1(){
  digitalWrite(motorPin1,LOW);
  digitalWrite(motorPin2,HIGH);
  Serial.println("DOOOOOOOOOOR");
  while (digitalRead(limitSwitch_2) == HIGH) {
    Serial.println(digitalRead(limitSwitch_2));
    // Wait until the limit switch is triggered
  }

  // Once the limit switch is triggered, stop the motor
  stopMotor_1();

  Serial.println("Motor stopped by limit switch.");
}

void stopMotor_1() {
  // Stop the motor
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}
void startMotor_2() {
  // Rotate motor in one direction
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
  Serial.println(digitalRead(limitSwitch_3));
  while (digitalRead(limitSwitch_3) == HIGH) {
    Serial.println("Door open");
  }

  // Once the limit switch is triggered, stop the motor
  stopMotor_2();

  Serial.println("Motor stopped by limit switch.");
}

void startrevMotor_2(){
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4,HIGH);
  while (digitalRead(limitSwitch_4) == HIGH) {
   Serial.println("Door Close");
  }

  // Once the limit switch is triggered, stop the motor
  stopMotor_2();

  Serial.println("Motor stopped by limit switch.");
}

void stopMotor_2() {
  // Stop the motor
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}
void startMotor_3() {
  // Rotate motor in one direction
  digitalWrite(motorPin5, LOW);
  digitalWrite(motorPin6, HIGH);
  while (digitalRead(limitSwitch_5) == HIGH) {
     Serial.println("Door open");
    // Wait until the limit switch is triggered
  }

  // Once the limit switch is triggered, stop the motor
  stopMotor_3();

  Serial.println("Motor stopped by limit switch.");
}
 //reversing the move of the motor
void startrevMotor_3(){
  digitalWrite(motorPin5, HIGH);
  digitalWrite(motorPin6,LOW);
  while (digitalRead(limitSwitch_6) == HIGH) {
    Serial.println("Door open");

    // Wait until the limit switch is triggered
  }

  // Once the limit switch is triggered, stop the motor
  stopMotor_3();

  Serial.println("Motor stopped by limit switch.");
}

void stopMotor_3() {
  // Stop the motor
  digitalWrite(motorPin5, LOW);
  digitalWrite(motorPin6, LOW);
}

// load cell code
void loadcell(){
  if (scale.is_ready()) {
    float weight = scale.get_units(10); 
    while(weight>100){
      weight = scale.get_units(10);
      Serial.println("over weight");
      myServo.write(90);

    }
  }

  delay(1000); // Wait for 1 second before next reading
}
void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
}
// Interrupt service routine for entering the interrupt state

void enterInterrupt() {
  interruptActive = true;  // Set the flag to indicate interrupt is active
  Serial.println("Interrupt entered.");
}

// Interrupt service routine for exiting the interrupt state
void exitInterrupt() {
  interruptActive = false;  // Clear the flag to stop the interrupt
  Serial.println("Interrupt exited.");
}