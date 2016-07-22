//Import all required libraries
#include <LiquidCrystal.h>
#include <Servo.h>
#include <GPRS_Shield_Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Suli.h>
#include <string.h>

/**
 * Pin Assignments &
 * Global Variable Declarations
 */
const int linePin = 3;      //Switch pin 
const int gprsPin = 2;      //Gprs pin
const int E1 = 5;           //First Motor speed
const int M1 = 4;           //First Motor Direction
const int E2 = 6;           //Second Motor speed             
const int M2 = 22;          //Second Motor Direction
const int servoPin = 9;     //Servo Motor 
const int irPinLeft = 2;    //Left optical sensor
const int irPinCenter = 1;  //Center optical sensor
const int irPinRight = 0;   //Right optical sensor
const int trigPin =  12;    //Trigger pin for sonar
const int echoPin =  13;    //Echo pin for sonar
const int tempPin =  4;     //Temperture pin for temperature sesnor
const int leftHall = 30;    //Left hall effect sensor
const int rightHall = 31;   //Righty hall effect sensor
const int backlightPin = 33;//Backlight for the lcd
const int RS  = 29;         //RS pin for the lcd
const int E   = 28;         //E pin for the lcd
const int DB4 = 27;         //Data line 4 for the lcd
const int DB5 = 26;         //Data line 5 for the lcd
const int DB6 = 25;         //Data line 6 for the lcd
const int DB7 = 24;         //Data line 7 for the lcd
      int waiting = 1;      //Whether or not we are waiting for an sms   

const int baseSpeed = 255;                    //Default speed
const int rightBaseSpeed = baseSpeed;         //Right motor speed
      int leftBaseSpeed = baseSpeed;          //Left motor speed
      int dynamicSpeedRight, dynamicSpeedLeft;//Dynamic speed for both motors to be used for deceleration
const int slowSpeed = 0;                      //Stopped speed
const int turnSpeed = 75;                     //Speed to turn at
const int detectedThresh = 50;                //Distance threshold to start slowing down   
const int dangerThresh = 15;                  //Distance to stop ats
const int turnTime = 625;                     //The amount of time to turn for
      int frontDistance, rightDistance, leftDistance, dynamicSpeed;//The distances on either side and in front of the robot


const int diffThresh = 40;     //Difference threshhold between the optical sensors 
      int irSensorLeft = 0;    //Left optical sensor reading
      int irSensorCenter = 0;  //Center optical sensor reading
      int irSensorRight = 0;   //Right optical sensor reading

// [0] is for the left wheel and [1] is for the right wheel
      int lastState[2] = { 0 }; //array for last hall effect state of both motors
      int currState[2] = { 0 }; //array for current hall effect state of both motors
      int startTime[2] = { millis(), millis() }; //array to store start times for both motors
      int totalTime[2] = { 1, 1 };   //aray to hold total amount time for each motor

      // variables related to the GPRS 
      #define MESSAGE_LENGTH 160   //Max sms text length
      char message[MESSAGE_LENGTH]; //Char array to hold sms 
      int messageIndex = 0;   //The current text being read
      int incoming = 0; //Whether there is a message to read or not
      char gprsBuffer[64]; //buffer for incoming messages
      char phone[16];     //The origin phone number from which the instruction came from
      char datetime[24];  //The timestamp for a message
      char got_your_command[] = "Got your command!"; //The response to the reception of a message

      #define PIN_TX    10  //Transmitting pin for gprs
      #define PIN_RX    11  //Receiving pin for gprs
      #define BAUDRATE  9600 //Baudrate for serial communications

const int baseSpeedGPRS = 150;           //The default speed for GPRS operation 
const int baseSpeedRightGPRS = baseSpeed;//The right motor speed for GPRS operation
const int baseSpeedLeftGPRS = baseSpeed; //The left motor speed for GPRS operation

      // this struct represents and instruction sent from cellular.
      typedef struct {
        char dir = 's';   //the direction for a specific instruction, default set to stop
        int duration = 50;//The duration for a specific instruction, default set to 50ms
      } instruction;

// the queue FIFO queue and Index
const int QUEUELENGTH = 10;  //the max number of instructions to be allowed into the queue at one time
      instruction queue[QUEUELENGTH]; //An queue holding all of the instructions to be executed  
      int queueIndex = 0;  //the intial queue index

// functional Objects
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7); //The lcd 
Servo sweepServo; //The servo
GPRS gprsTest(PIN_TX,PIN_RX,BAUDRATE);//The gprs shield

/**
 * The main purpose of the setup() function is to initialize all the things
 * the robot needs to operate on every mode. This includes setting up:
 * - the pins
 * - the servo
 * - the lcd screen
 * - the GPRS shield cellular
 */
void setup() {
    Serial.begin(9600); // initialize serial communication at 9600 bits per second:

    sweepServo.attach(servoPin); //Attach the servo to the the servo pin

    //Set both motors to Output
    pinMode(M1, OUTPUT);   
    pinMode(M2, OUTPUT);
    
    pinMode(linePin, INPUT); //set the pin for the line follow mode switch to input
    pinMode(gprsPin, INPUT);//set the pin for the gprs mode switch to input

    pinMode(trigPin, OUTPUT);//Set the trigger pin for the sonar to output
    //Set all other utilized pins to input
    pinMode(echoPin, INPUT); //Set the echo pin for the sonar to input
    pinMode(tempPin, INPUT); //Set the temperature pin for the sonar to input
    
    pinMode(leftHall, INPUT); //Set the left hall effect sensor to input
    pinMode(rightHall, INPUT);//Set the right hall effect sensor to input

    pinMode(backlightPin, OUTPUT); //Set the backlight pin for the lcd to output 

    toggleBacklight(0); //turn the backlight off initially

    if(digitalRead(gprsPin) == HIGH){ //If the mode if initially in gprs 
       lcd.begin(16, 2);          //initialize the lcd
       lcd.print("Initializing"); //Print that the setup is executing to the lcd
       lcd.setCursor(0, 1);       //Move to the second line
       lcd.print("GPRS Shield...");//Continuation of the message communicating setup
      while(0 != gprsTest.init()) { //while the gprs shield is not ready
         delay(1000); //wait for a second
         Serial.print("init error\r\n"); //Signal that there was an error to the serial monitor

          lcd.clear();//Clear the lcd
          lcd.setCursor(0, 0); //Set the cursor to 0
          lcd.print("init error"); //Signal that there was an error to the lcd
       }

       delay(3000);  //wait for 3s
       Serial.println("Init Success, please send SMS message to me!"); //Signal that the setup is complete

       lcd.clear(); //clear the lcd
       lcd.setCursor(0, 0); // set the cursor to the top row
       lcd.print("Setup complete!");//Show that the setup is done to the lcd
       delay(2000); //delay 
    }
}

/**
 * The main function of the robot's functionality.
 * Based on the function switch in the robot itself,
 * it will execute the:
 * - Line Following           or
 * - Cellular functionality   or
 * - Object Avoidance
 */
void loop() { 

  // follow the line if the switch is one direction
  if (digitalRead(linePin) == HIGH){
      toggleBacklight(0); //turn off the backlight
      lineFollow();//Execute line following while in line follow mode
    
  }
  // use the additional functionality if the switch is the other direction
  else if (digitalRead(gprsPin) == HIGH){
      toggleBacklight(1);//turn on the backlight
      gprs();  //Execute the gprs mode while the gprspin is high
  }
  // if the switch is in the middle use the ultrasonic range finder
  else{
      toggleBacklight(0);//turn off the backlight
      objAvoid();//Execute the object avoidance program while in objAvoid mode
  }
    
}

///////////////////OBJECT AVOIDING SECTION///////////////////////

/**
 * The function that avoids obstacles using the ultrasonic
 * range finding sensors. Compares the distance with the
 * front of the robot. If its far enough, then just continue
 * going. If not, stop and sweep to the sides. It will turn
 * and go to a side depending on which distance is less.
 */
void objAvoid(){

  // adjust the wheels based on the hall effect sensors
  //hallAdjust();

  // get the current distance from the ultrasonic range finder
  frontDistance = getDistance();
  // when you get within detectedThresh, start slowing down up until dangerThresh
  if(frontDistance <= detectedThresh){
    // adjust the speed based on the distance from the object
    dynamicSpeedRight = map(frontDistance, dangerThresh, detectedThresh, rightBaseSpeed/3, rightBaseSpeed);
    dynamicSpeedLeft = map(frontDistance, dangerThresh, detectedThresh, leftBaseSpeed/3, leftBaseSpeed);

     // continue going straight
    goStraight(dynamicSpeedRight, dynamicSpeedLeft);

    // sweep the area once you get to the dangerThresh
    if(frontDistance <= dangerThresh + 10){
      sweep();
    }
  }
  // otherwise just go straight with the current adjusted left and right base speeds
  else{
    goStraight(rightBaseSpeed, leftBaseSpeed);
  }
  
}

/**
 * Hall effect sensors code. Adjusts the speed of each wheel
 * depending on the speed of the other one, ultimately to
 * make it go relatively more straight.
 */
void hallAdjust(){
  // left wheel
  currState[0] = digitalRead(leftHall);
  // if the hall effect sensor changes to a 0
  if (currState[0] == 0 && lastState[0] == 1) {
    // calculate the total time for one revolution of the wheel
    totalTime[0] = millis() - startTime[0]; //Update the value of the total time for the first motor
    startTime[0] = millis(); //update the value of the start time for the first motor
  }
  // set the last state for the left wheel
  lastState[0] = currState[0];

  // right wheel
  currState[1] = digitalRead(rightHall);
  // if the hall effect sensor changes to a 0
  if (currState[1] == 0 && lastState[1] == 1) {
    // calculate the total time for one revolution of the wheel
    totalTime[1] = millis() - startTime[1];//Update the value of the total time for the second motor
    startTime[1] = millis(); //update the value of the start time for the second motor
  }
  // set the last state for the right wheel
  lastState[1] = currState[1];

  //Set the left speed to be a function of the ratio of roation times
  leftBaseSpeed = leftBaseSpeed * totalTime[1] / totalTime[0];
  analogWrite(E1, leftBaseSpeed); //set the adjusted speed
  
}

/**
 * Helper function for objAvoid() function. It sweeps the servo so
 * the sonar sensor can scan each side of the robot. As defense for
 * incorrect readings, we use a get median function to compare three
 * readings from the sensor (in case it gets an incorrect large
 * reading or an incorrect low reading).
 */
void sweep() {

  stopWheels(); //Stop the wheels
  delay(500);   //Wait for 500ms

  sweepServo.write(0); //Set the servo to look right
  delay(1000);         //wait for a second
  rightDistance = getMedian(getDistance(), getDistance(), getDistance()); //Get the right distance
  delay(1000);         //wait for a second

  sweepServo.write(180); //Set the servo to look left
  delay(1000);//wait for a second
  leftDistance = getMedian(getDistance(), getDistance(), getDistance()); //Get the left distance
  delay(1000);//wait for a second

  sweepServo.write(90); //Set the servo to look straight 
  delay(50);//wait for 50ms

  compareDistances();//Compare the measured distances to the left and right and decide where to go
  
}

/**
 * Gets the distance from the sensor to the nearest object. Also adjusts
 * that distance relative to the temperature sensor.
 */
long getDistance(){

  //Get the the current temperature reading from the temperature pin and 
  //perform the required offsets/conversions
  double tempReading = analogRead(tempPin) / 10.0 + 20.0;
  float duration, spdSnd;
  
  //Ensure that the trigger pin is low
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);

  //Put the trigger pin to high for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //Wait for the echoPin to send a response and set 'duration' to
  //be the time it took to receive the response
  duration = pulseIn(echoPin, HIGH);

  spdSnd = 331.5 + (0.6 * tempReading);
  
  //Serial.println((duration / 2) * (spdSnd / 10000));
  return (duration / 2) * (spdSnd / 10000);
  
}

/*
 * This function ultimately decides in which way the robot will turn
 * given the distances for objects on the right side
 * and the left side.
 */
void compareDistances(){

  if (leftDistance < rightDistance) //if right is a better option 
  {
    
    goRight(); //turn to the right
    delay(turnTime);  //Wait for the amount of time required to turn 90 degrees
    
  }
  else //f(rightDistance < leftDistance) //if left is a better option 
  {

    goLeft(); //Turn to the left
    delay(turnTime); //Wait for the amount of time required to turn 90 degrees
 
  }
  
}

//////////////////////LINE FOLLOWING SECTION////////////////////

/**
 * This function implements the second functionality of our robot, which
 * is to follow a black line on the floor. If it reads the right sensor,
 * it will go right, else if it read the center sensor it will go straight, 
 * else if it read the left sensor it will go left, otherwise just straight.
 */
void lineFollow(){

  readIrSensors(); //Get the current optical sensor readings
  
  if (irSensorRight == 1) { //If the right sensor is triggered
    
    goRight();  //Go right
    
  } else if (irSensorCenter == 1) { //If the center sensor is triggered

    goStraight(baseSpeed, baseSpeed); //Go straight at the speed of baseSpeed
    
  } else if (irSensorLeft == 1) { //If the left sensor is triggered

    goLeft(); //Go left 

  } else { // (defense) [GO STRAIGHT]

    goStraight(baseSpeed, baseSpeed); //Go straight at the speed of baseSpeed
    
  }

  delay(10); //Wait for 10ms
}

/** 
 * Read all of the IR sensors on the bottom of the robot
 * and discretizes their values to a 0 or a 1, where 0
 * is a white surface and 1 is a black surface.
 */
void readIrSensors() {
     
    readCenterSensor(); //Get the center optical sensor reading
    readRightSensor();  //Get the right optical sensor reading
    readLeftSensor();   //Get the left optical sensor reading
    
    discretizeSensors();//Discretive the sensor readings
    delay(5); //wait for 5ms
    
}
/**
 * Reads the center sensor.
 */
void readCenterSensor(){
  
   irSensorCenter = analogRead(irPinCenter); //Read the center optial sensor pin
   delay(5); // delay 5ms
   
}

/**
 * Reads the left sensor.
 */
void readLeftSensor(){
  
  irSensorLeft = analogRead(irPinLeft);//Read the left optial sensor pin
  delay(5); // delay 5ms
  
}

/**
 * Reads the right sensor.
 */
void readRightSensor(){
  
  irSensorRight = analogRead(irPinRight);//Read the right optial sensor pin
  delay(5); // delay 5ms
  
}

///////////////////////GPRS SECTION/////////////////////

/**
 * This function implements the main part of the cellular
 * ADDITIONAL FUNCTIONALITY of our robot. The function loops
 * and awaits a text message from the user and then calls 
 * a function to parse the message to give the robot instructions.
 * The format of a useful message you can send to the robot is as follows:
 * 
 * @#;@#;@#;@#;@#;@#;@#;@#;@#; .............
 * 
 * where @ is the direction (f = forward, b = backward, l = left,
 * r = right, s = stop) and #'s are any amount of digits you wish. 
 * The direction tells the robot in which direction to go/turn and the number
 * describes, in milliseconds, how long it will do that. You can send one or
 * more instructions at a time, as long as they are followed by a semicolon.
 */
void gprs() {

  // If the machine is awaiting an SMS
  if (waiting) {
    lcd.clear(); //Clear the lcd
    lcd.setCursor(0, 0); //set the cursor to the top row
    lcd.print("Awaiting your"); //Communicate that the robot is awaiting a command
    lcd.setCursor(0, 1); //Set the cursor to the second row
    lcd.print("command..."); //Continues message to Communicate that the robot is awaiting a command
    waiting = 0; //Set waiting to 0
  }
  
   messageIndex = gprsTest.isSMSunread(); // See if the there are unread messages (library implemented)
   if (messageIndex > 0) { //At least, there is one UNREAD SMS (library implemented)
      waiting = 1; //Set waiting to 1
      lcd.clear(); //Clear the lcd
      lcd.setCursor(0, 0); //Set the cursor to the top row
      lcd.print("Receiving SMS..."); //Print to the lcd that an sms is being received
      gprsTest.readSMS(messageIndex, message, MESSAGE_LENGTH, phone, datetime); //Read the incoming sms and store the information associated with it        
      gprsTest.deleteSMS(messageIndex);//In order not to full SIM Memory, it is better to delete the SMS after it's been received
      parseMessage(message); // Main function for functionality given a message.
      gprsTest.sendSMS(phone, got_your_command); // text the user with the cellphone that the text message has been received
      delay(500);//wait for half a second                            

   }

   while(queueIndex > 0) executeInstruction(); // execute instructions in the FIFO style instruction queue.
    
}

/**
 * Parses the message and puts the information to the instruction queue.
 */
void parseMessage(char* message){
  int lastSemicolonIndex = 0;//Initialize the index of the last semi colon to 0
  //Loop through the entire lenth of the message
  for (int i = 0; i < strlen(message); i++)
    if (message[i] == ';') //If the current element in the char array is a semi colon
      lastSemicolonIndex = i;//Update the index of the last semi colon
  int lastindex = 0; //Initialize the index of the last instruction to 0
  for (int index = 0; index <= lastSemicolonIndex; index++) {
    if (message[index] == ';') { //If the current element in the char array is a semi colon
      char* newArray = makeNewCharArray(message, lastindex, index);//create new char array that is a subset of the original message
      parseInstruction(newArray);//Parse the new char array using parseInstruction
      free(newArray);//Free the memory allocated for the new char array, as it is no longer needed
      newArray = NULL; //Set the newArray to Null 
      lastindex = index+1; //Update the index of the last instruction within the message
    }
  }
}

/**
 * Helper function to parse each individual instruction to add it
 * to the queue, and raise the queue index by one.
 */
void parseInstruction(char* command) {
  queue[queueIndex].dir = command[0]; //Set the direction of the instruction at queueIndex to be the first char of command
  queue[queueIndex].duration = 0; //Initialize the duration of the instruction at queueIndex to 0
  //Loop through command starting from the last element 
  for (int i = strlen(command) - 1, powTen = 1; i > 0; i--, powTen *= 10) //Decrement i and multiply powTen by 10 on every iteration
    queue[queueIndex].duration += (command[i]-48)*powTen; //Add the value of the current int being analyzed multiplied by 10^x, where x is the current position in the number
  queueIndex++;//Increment the queue Index as a new instruction was just added
}

/**
 * Helper function to separate each instruction from a text message
 * that could possibly contain more than one instruction.
 * 
 * %---% including left index and excluding right index %---%
 */
char* makeNewCharArray(char* old, int left, int right) {
  char *newArray = (char *) malloc((right-left+1)*sizeof(char)); //Allocate space for the new char array depending on the size of the instruction
  int j = 0; //Initialize j to 0, this will be that variable used to loop through the new array 
  //Loop through the subset of the "old" array to transfer it to the new array
  for (int i = left; i < right; i++, j++) { //Increment j and i on every iteration
    newArray[j] = old[i]; //Set the newArray at j to be the old array at i
  }
  newArray[j] = '\0'; //Set the last element of the new array to the null char
  return newArray; //Return the newly created char array
}

/**
 * Actually executes each instruction in the queue, popping its elements
 * one by one and gives instructions to the robot to do them.
 */
void executeInstruction() {
  instruction exec = popFromQueue(); //Get the current instruction from the instruction queue
  lcd.clear(); //clear the lcd
  lcd.setCursor(0, 0);//set the cursor to the first line

  //use a switch to see which instruction is being called
  switch(exec.dir){ 
    case 'f':  //If the direction is forwards
              lcd.print("Forwards:"); //Print the current direction on the lcd
              goStraight(baseSpeedRightGPRS, baseSpeedLeftGPRS); //Go straight
              break;
    case 'b': //If the direction is backwards
              lcd.print("Backwards:");//Print the current direction on the lcd
              goBack();//Go backwards
              break;
    case 'r': //If the direction is right
              lcd.print("Right:");//Print the current direction on the lcd
              goRight();//Go right
              break;
    case 'l': //If the direction is left
              lcd.print("Left:");//Print the current direction on the lcd
              goLeft();//Go left
              break;  
    case 's': //If the direction is stop
              lcd.print("Stop:");//Print the current direction on the lcd
              stopWheels();//Stop the wheels
              break;           
    default:  //If the instruction is invalid
              lcd.print("Stop:");//Print the current direction on the lcd
              stopWheels(); //Stop the wheels  
              break;      
  }
  lcd.setCursor(0, 1);//Set the cursor to second row
  lcd.print(exec.duration); //print the duration of the current instruction on the lcd
  lcd.print("ms"); //Print the units of the duration onto the lcd
  delay(exec.duration);//wait for the correct amount of delay time
}

/**
 * Returns the leading element in the queue, and shifts all other elements down by one.
 */
instruction popFromQueue() {
  instruction ret = queue[0]; //set the instruction to be returned to the first one
  //Shift the entire queue down so that the next instruction is lined up
  for(int i = 0; i < QUEUELENGTH - 1; i++){
    queue[i].dir = queue[i+1].dir; 
    queue[i].duration = queue[i+1].duration;
  }
  queue[QUEUELENGTH - 1].dir = 's'; //Set the last element to have a direction of 'stop'
  queue[QUEUELENGTH - 1].duration = 50; //Set the last element to have a duration of 50ms
  queueIndex--; //decrement the queue index
  return ret;//return the first instruction in the queue
}

/**
 * clears the queue and sets all elements to default values.
 */
void clearQueue() {
  //Loop through the queue to clear it2
  for (int i = 0; i < QUEUELENGTH; i++) { 
    queue[i].dir = 's'; //Set every instruction in the queue to have a direction of stop
    queue[i].duration = 50; //Set every instruction in the queue to have a duration of 50ms
  }
  queueIndex = 0; //Reset the queue index to 0
}

/**
 * Makes the sensor readings discrete.
 * 1 means it's reading a black surface (for the tape)
 * 0 means it's reading a white surface (for the floor)
 */
void discretizeSensors() {
  
  irSensorLeft = (irSensorLeft >= 900)      ? 1 : 0;  //If the left sensor if above its calibrated threshhold, return 1
  irSensorCenter = (irSensorCenter >= 900)  ? 1 : 0;  //If the center sensor if above its calibrated threshhold, return 1
  irSensorRight = (irSensorRight >= 875)    ? 1 : 0; //If the right sensor if above its calibrated threshhold, return 1
  
}

/**
 * Gets the median of three longs.
 */
long getMedian(long dist1, long dist2, long dist3){
  
  long x = dist1 - dist2;   //set x to be the difference between the first distance and the second
  long y = dist2 - dist3;   //set y to be the difference between the second distance and the third
  long z = dist1 - dist3;   //set y to be the difference between the first distance and the third
  if(x*y > 0) return dist2; //If x * y if greater than zero, return that the second distance is the median
  if(x*z > 0) return dist3; //If x * z if greater than zero, return that the third distance is the median
  return dist1;             //Otherwise the first distance is the median and return it.
 
}

//////MOVING FUNCTIONS//////

//This function stops the robot
void stopWheels(){

    //Set the left motor to have a speed of zero
    digitalWrite(M2, HIGH);
    analogWrite(E2, 0);

    //Set the right motor to have a speed of zero
    digitalWrite(M1, HIGH);
    analogWrite(E1, 0);

}

//This function nakes the robot goes straight
void goStraight(int rightSpd, int leftSpd){

    //Set the right motor to have a speed of rightSpd in the forward direction
    digitalWrite(M1, HIGH);
    analogWrite(E1, rightSpd);

    //Set the left motor to have a speed of leftSpd in the forward direction
    digitalWrite(M2, HIGH);
    analogWrite(E2, leftSpd);
  
}

//This function make the robot goes backwards
void goBack(){

    //Set the right motor to have a speed of baseSpeed in the backwards direction
    digitalWrite(M1, LOW);
    analogWrite(E1, baseSpeed);

    //Set the left motor to have a speed of baseSpeed in the backwards direction
    digitalWrite(M2, LOW);
    analogWrite(E2, baseSpeed);
  
}

//This function turns the robot to the left
void goLeft() {  

      //Set the left motor to have a speed of baseSpeed in the forwards direction
     digitalWrite(M2, HIGH);
     analogWrite(E2, baseSpeed);

     //Set the right motor to have a speed of baseSpeed in the backwards direction
     digitalWrite(M1, LOW);
     analogWrite(E1, baseSpeed);    
     
}

//This function turns the robot to the right
void goRight() {

    //Set the right motor to have a speed of baseSpeed in the forwards direction
    digitalWrite(M1, HIGH);
    analogWrite(E1, baseSpeed);

    //Set the left motor to have a speed of baseSpeed in the backwards direction
    digitalWrite(M2, LOW);
    analogWrite(E2, baseSpeed);
     
}

//This function turns on/off the lcd backlight
void toggleBacklight(int on){

  //if we are to turn the lcd backlight on
  if(on == 1){
    digitalWrite(backlightPin, HIGH); //Set the lcd backlight pin to high
    
  }else{
    digitalWrite(backlightPin, LOW); //Otherwise set the lcd backlight pin to low
  }
  
}



