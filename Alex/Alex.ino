#include <serialize.h>
#include <math.h>

#include "packet.h"
#include "constants.h"

volatile bool overrideIR = true;
volatile bool manualOverride = false;
bool pcint_negedge = false;

typedef enum {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
   Alex's configuration constants
*/

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV      192.0

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.42035

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  11  // Right forward pin
#define RR                  10  // Right reverse pin

#define PI                  3.14159264 

// Alex's length and breadth (cm)
#define ALEX_LENGTH 8.0
#define ALEX_BREADTH 5.5

// Alex's diagonal, we compute and store this value only once
// since it is expensive to compute and doesn't really change.
float AlexDiagonal = 0.0;

//Alex's turning circumference
float AlexCirc = 0.0;

/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Counters for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile float forwardDist;
volatile float reverseDist;

// Variables to keep track of whether we have moved a commanded dist
float deltaDist;
float newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

//Variables to keep track of IR sensor input
volatile unsigned long leftIr;
volatile unsigned long middleIr;
volatile unsigned long rightIr;
volatile bool leftIrTrigger;
volatile bool middleIRTrigger;
volatile bool rightIRTrigger;

/*

   Alex Communication Routines.

*/

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;  
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pin 2
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2. This corresponds to pin PD2.
  // We set bits 2 in DDRD to 0 to make them inputs.

  // MODIFICATION: Enable only pin 2 pull up since we are using PCINT4 as replacement

  DDRD  &= 0b11111011;
  PORTD |= 0b00000100;

  // Enable pin 12 pull up for PCINT4 interrupt detection
  DDRB  &= 0b11101111;
  PORTB |= 0b00010000;
}

// Functions to be called by INT0 ISRs.
void leftISR()
{
  switch (dir) {
    case FORWARD:
      leftForwardTicks++;
      forwardDist = (((float) leftForwardTicks) / COUNTS_PER_REV * WHEEL_CIRC);
      break;

    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (((float) leftReverseTicks) / COUNTS_PER_REV * WHEEL_CIRC);
      break;

    case LEFT:
      leftReverseTicksTurns++;
      break;

    case RIGHT:
      leftForwardTicksTurns++;
      break;
  }

  // We calculate forwardDist only in leftISR because we
  // assume that the left and right wheels move at the same
  // time.

  Serial.print("Left ticks =");
  Serial.print(" ");
  Serial.print(leftForwardTicks);
}

void rightISR()
{
  pcint_negedge = PINB & 0b00010000; // Since PCINT occurs at any pin change, and we only require falling edge
  if (pcint_negedge){ //if pin is reading high after interrupt is triggered, not negedge
    return ;
  }
  switch (dir) {
    case FORWARD:
      rightForwardTicks++;
      break;

    case BACKWARD:
      rightReverseTicks++;
      break;

    case LEFT:
      rightForwardTicksTurns++;
      break;

    case RIGHT:
      rightReverseTicksTurns++;
      break;    
  }
  
  rightRevs = ((float) rightForwardTicks) / COUNTS_PER_REV;
  Serial.print("RIGHT ticks: ");
  Serial.println(rightForwardTicks);
}

  // Set up the external interrupt pins INT0
  // for falling edge triggered. Use bare-metal.
  void setupEINT()
  {
    // Use bare-metal to configure pins 2 to be
    // falling edge triggered. Remember to enable
    // the INT0 interrupt.

    //MODIFIED: ONLY INT0 enabled

    EICRA = 0b00001010;
    EIMSK = 0b00000001;

    PCICR |= 0b00000001;
    PCMSK0 |= 0b00010000;
  }

  // Implement the external interrupt ISRs below.
  // INT0 ISR should call leftISR

  ISR(INT0_vect)
  {
    leftISR();
  }

  ISR(PCINT0_vect) //PCINT0 controls PCINT4 (PIN 12) INTERRUPT
  {
    rightISR();
  }

  /*
     Setup and start codes for serial communications

  */
  // Set up the serial connection. For now we are using
  // Arduino Wiring, you will replace this later
  // with bare-metal code.
  void setupSerial()
  {
    // To replace later with bare-metal.
    Serial.begin(57600);
  }

  // Start the serial connection. For now we are using
  // Arduino wiring and this function is empty. We will
  // replace this later with bare-metal code.

  void startSerial()
  {
    // Empty for now. To be replaced with bare-metal code
    // later on.
  }

  // Read the serial port. Returns the read character in
  // ch if available. Also returns TRUE if ch is valid.
  // This will be replaced later with bare-metal code.

  int readSerial(char *buffer)
  {

    int count = 0;

    while (Serial.available())
      buffer[count++] = Serial.read();

    return count;
  }

  // Write to the serial port. Replaced later with
  // bare-metal code

  void writeSerial(const char *buffer, int len)
  {
    Serial.write(buffer, len);
  }

  /*
     Alex's motor drivers.

  */

  // Set up Alex's motors. Right now this is empty, but
  // later you will replace it with code to set up the PWMs
  // to drive the motors.
  void setupMotors()
  {

    /* Our motor set up is:
          A1IN - Pin 5, PD5, OC0B
          A2IN - Pin 6, PD6, OC0A
          B1IN - Pin 10, PB2, OC1B
          B2In - pIN 11, PB3, OC2A
    */
  }

  // Start the PWM for Alex's motors.
  // We will implement this later. For now it is
  // blank.
  void startMotors()
  {

  }

  // Convert percentages to PWM values
  int pwmVal(float speed)
  {
    if (speed < 0.0)
      speed = 0;

    if (speed > 100.0)
      speed = 100.0;

    return (int) ((speed / 100.0) * 255.0);
  }

  // Move Alex forward "dist" cm at speed "speed".
  // "speed" is expressed as a percentage. E.g. 50 is
  // move forward at half speed.
  // Specifying a distance of 0 means Alex will
  // continue moving forward indefinitely.
  void forward(float dist, float speed)
  {
    deltaDist = (dist == 0)? 9999999 : dist;
    newDist = forwardDist + deltaDist;    
    dir = FORWARD;
    int val = pwmVal(speed);
    
    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.
    analogWrite(LF, val);
    analogWrite(RF, 0.865 * val);
    analogWrite(LR, 0);
    analogWrite(RR, 0);
  }

  // Reverse Alex "dist" cm at speed "speed".
  // "speed" is expressed as a percentage. E.g. 50 is
  // reverse at half speed.
  // Specifying a distance of 0 means Alex will
  // continue reversing indefinitely.
  void reverse(float dist, float speed)
  {
    
    deltaDist = (dist == 0)? 9999999 : dist;
    newDist = reverseDist + deltaDist;
    dir = BACKWARD;
    int val = pwmVal(speed);
    // For now we will ignore dist and
    // reverse indefinitely. We will fix this
    // in Week 9.

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.
    analogWrite(LR, val);
    analogWrite(RR, val);
    analogWrite(LF, 0);
    analogWrite(RF, 0);
  }

  unsigned long computeDeltaTicks(float ang){
    // We will assume that the angular distance moved = linear distance moved in one wheel
    // revolution. This is (probably) incorrect but simplifies calculation.
    // # of wheel revs to make one full 360 turn is AlexCirc/ WHEEL_CIRC
    // This is for 360 degrees. For ang degrees it will be (ang * AlexCirc) / (360 * WHEEL_CIRC)
    // To convert to ticks, we multiply by COUNTS_PER_REV.
    return (unsigned long)((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  }

  // Turn Alex left "ang" degrees at speed "speed".
  // "speed" is expressed as a percentage. E.g. 50 is
  // turn left at half speed.
  // Specifying an angle of 0 degrees will cause Alex to
  // turn left indefinitely.
  void left(float ang, float speed)
  {
    deltaTicks = (ang == 0)? 99999999: computeDeltaTicks(ang);
    targetTicks = leftReverseTicksTurns + deltaTicks;
    dir = LEFT;
    
    int val = pwmVal(speed);

    // For now we will ignore ang. We will fix this in Week 9.
    // We will also replace this code with bare-metal later.
    // To turn left we reverse the left wheel and move
    // the right wheel forward.
    analogWrite(LR, val);
    analogWrite(RF, val);
    analogWrite(LF, 0);
    analogWrite(RR, 0);
  }

  // Turn Alex right "ang" degrees at speed "speed".
  // "speed" is expressed as a percentage. E.g. 50 is
  // turn left at half speed.
  // Specifying an angle of 0 degrees will cause Alex to
  // turn s indefinitely.
  void right(float ang, float speed)
  {
    deltaTicks = (ang == 0)? 99999999: computeDeltaTicks(ang);
    targetTicks = rightReverseTicksTurns + deltaTicks;
    dir = RIGHT;
    int val = pwmVal(speed);

    // For now we will ignore ang. We will fix this in Week 9.
    // We will also replace this code with bare-metal later.
    // To turn right we reverse the right wheel and move
    // the left wheel forward.
    analogWrite(RR, val);
    analogWrite(LF, val);
    analogWrite(LR, 0);
    analogWrite(RF, 0);
  }

  // Stop Alex. To replace with bare-metal code later.
  void stop()
  {
    dir = STOP;
    analogWrite(LF, 0);
    analogWrite(LR, 0);
    analogWrite(RF, 0);
    analogWrite(RR, 0);
  }

  /*
     Alex's setup and run codes

  */

  // Clears all our counters
  void clearCounters()
  {
    leftForwardTicks = 0;
    leftReverseTicks = 0;
    rightForwardTicks = 0;
    rightReverseTicks = 0;
    leftForwardTicksTurns = 0;
    leftReverseTicksTurns = 0;
    rightForwardTicksTurns = 0;
    rightReverseTicksTurns = 0;
    leftRevs = 0;
    rightRevs = 0;
    forwardDist = 0;
    reverseDist = 0;
  }

  // Clears one particular counter
  void clearOneCounter(int which)
  {
//      switch(which)
//      {
//        case 0:
//          clearCounters();
//          break;
//    
//        case 1:
//          leftTicks=0;
//          break;
//    
//        case 2:
//          rightTicks=0;
//          break;
//    
//        case 3:
//          leftRevs=0;
//          break;
//    
//        case 4:
//          rightRevs=0;
//          break;
//    
//        case 5:
//          forwardDist=0;
//          break;
//    
//        case 6:
//          reverseDist=0;
//          break;
//      }
  }
  
  // Intialize Alex's internal states
  void initializeState()
  {
    clearCounters();
  }

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
        break;
      
    case COMMAND_OVERRIDEIR:
      overrideIR = !overrideIR;
      (overrideIR)? sendMessage("override on") : sendMessage("override off");
      break;
      
    case COMMAND_SCAN_COLOUR:
      stop();
      colourDetect();
      break;
           
    default:
      sendBadCommand();
  }
}

  void waitForHello()
  {
    int exit = 0;

    while (!exit)
    {
      TPacket hello;
      TResult result;

      do
      {
        result = readPacket(&hello);
      } while (result == PACKET_INCOMPLETE);

      if (result == PACKET_OK)
      {
        if (hello.packetType == PACKET_TYPE_HELLO)
        {
          sendOK();
          exit = 1;
        }
        else
          sendBadResponse();
      }
      else if (result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else if (result == PACKET_CHECKSUM_BAD)
        sendBadChecksum();
    } // !exit
  }

void setupADC(){
  PRR &= 0b11111110; // Turn on ADC power 
  ADCSRA = 0b10001111; // Enable ADC module, completion interrupt, clock divider 128
  ADMUX = 0b01000000; // initialize multiplexer to A0
}
  
void startADC(){
  ADCSRA |= 0b01000000; // start ADC conversion
}



//
//    IR SENSOR ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
ISR(ADC_vect){ //Interrupt triggered upon completion of analog to digital conversion
    unsigned int loval = ADCL;
    unsigned int hival = ADCH;
    unsigned int adcvalue = hival * 256 + loval; // ADC conversion result is 10 bits, stored in ADCH & ADCL

  // Multiplex between the sensors
  // IR sensor is approximately digital (0 when too close)
  switch(ADMUX){ 
    case 0b01000000: // A0
    if (adcvalue < 500 && !(overrideIR)){
      overrideIR = true;
      stop();
      delay(10);
      sendMessage("L on");
    }
      ADMUX = 0b01000001;
      break;
    
    case 0b01000001: // A1
      if (adcvalue < 500 && !(overrideIR)){
        overrideIR = true;
        stop();
        delay(10);
        sendMessage("C on");
      }
      ADMUX = 0b01000010;
      break;

    case 0b01000010: // A2
      if (adcvalue < 500 && !(overrideIR)){
        overrideIR = true;
        stop();
        delay(10);
        sendMessage("R on");
      }
      ADMUX = 0b01000000;
      break;
  }
  ADCSRA |= 0b01000000; // Restart ADC conversion
}

void setupColourSensor(){
  // Set S0:4, S3:7, S1:3, S2:9 as output
  // Set sensorOut:8 as input
  DDRD |= 0b10011000;
  DDRB |= 0b00000010;
  DDRB &= 0b11111110; 
  
  //power saver mode - set S0 and S1 as LOW
  PORTD &= 0b11100111;
}

void colourDetect(){
  int sensorOut = 8;
  int red, blue, green;

  // Set at 20% power - Set S0 as HIGH and S1 as LOW
  PORTD |= 0b00010000;
  PORTD &= 0b11110111;
  delay(300);
  //Serial.println("colour detector on");
  
  //Detect red colour - Set S2 and S3 as LOW
  PORTB &= 0b11111101;
  PORTD &= 0b01111111;  
  //delay(300);
  red = pulseIn(sensorOut, LOW);
  red = map(red, 0, 1023, 0, 255);
  
  //Detect green colour - Set S2 and S3 as HIGH
  PORTB |= 0b00000010;
  PORTD |= 0b10000000;
  //delay(300);
  green = pulseIn(sensorOut, LOW);
  green = map(green, 0, 1023, 0, 255);

  //Detect blue colour - Set S2 as low and S3 as HIGH
  PORTB &= 0b11111101;
  PORTD |= 0b10000000;
  //delay(300);
  blue = pulseIn(sensorOut, LOW);
  blue = map(blue, 0, 1023, 0, 255);

//  Serial.print("Red = ");
//  Serial.println(red);
// 
//  Serial.print("Green = ");
//  Serial.println(green);
// 
//  Serial.print("Blue = ");
//  Serial.println(blue);
//  
//  Serial.println((red < 550 && blue > red && green > red));

  //power saver mode
  PORTD &= 0b11101111;
  PORTB &= 0b11101111; 
  
  (red < 550 && blue > red && green > red) ? sendMessage("g") : sendMessage("r");
}

void setup() {
    // put your setup code here, to run once:
    // Compute the diagonal
    AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH *ALEX_BREADTH));
    AlexCirc = PI * AlexDiagonal;

    cli();
    setupEINT();
    setupSerial();
    startSerial();
    setupMotors();
    startMotors();
    enablePullups();
    initializeState();
    setupADC();
    setupColourSensor();
    sei();
    startADC();
    forward(100,75);
  }

void handlePacket(TPacket * packet)
  {
    switch (packet->packetType)
    {
      case PACKET_TYPE_COMMAND:
        handleCommand(packet);
        break;

      case PACKET_TYPE_RESPONSE:
        break;

      case PACKET_TYPE_ERROR:
        break;

      case PACKET_TYPE_MESSAGE:
        break;

      case PACKET_TYPE_HELLO:
        break;
    }
  }

  void loop() {
    // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

    // Uncomment the code below for Week 9 Studio 2

      // put your main code here, to run repeatedly:
      TPacket recvPacket; // This holds commands from the Pi

      TResult result = readPacket(&recvPacket);

      if(result == PACKET_OK)
        handlePacket(&recvPacket);
      else
        if(result == PACKET_BAD)
        {
          sendBadPacket();
        }
        else
          if(result == PACKET_CHECKSUM_BAD)
          {
            sendBadChecksum();
          }

      if(deltaDist > 0){
        if(dir==FORWARD && forwardDist >= newDist){
          deltaDist=0;
          newDist=0;
          stop();
        }
        
        else if(dir == BACKWARD && reverseDist >= newDist){
          deltaDist=0;
          newDist=0;
          stop();
        }
      
      else if(dir == STOP){
          deltaDist=0;
          newDist=0;
          stop();
          }
       }

      if (deltaTicks > 0){
        if (dir == LEFT && leftReverseTicksTurns >= targetTicks){
            deltaTicks = 0;
            targetTicks = 0;
            stop();
        }

        else if (dir == RIGHT && rightReverseTicksTurns >= targetTicks){
          deltaTicks = 0;
          targetTicks = 0;
          stop();  
        }

        else if (dir == STOP){
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      }
  }
