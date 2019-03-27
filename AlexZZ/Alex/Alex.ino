#include <serialize.h>

#include "packet.h"
#include "constants.h"

bool overrideIR = true; 

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

#define COUNTS_PER_REV      192

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.42035

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin

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
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;


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
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.

  DDRD  &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  leftForwardTicks++;

  switch (dir) {
    case FORWARD:
      leftForwardTicks++;
      forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;

    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;

    case LEFT:
      leftReverseTicksTurns++;
      break;

    case RIGHT:
      leftForwardTicksTurns++;
      break;
  }

  leftRevs = leftForwardTicks / COUNTS_PER_REV;

  // We calculate forwardDist only in leftISR because we
  // assume that the left and right wheels move at the same
  // time.
  forwardDist = leftRevs * WHEEL_CIRC;


  Serial.print("LEFT: ");
  Serial.println(leftForwardTicks);
}

void rightISR()
{
  rightForwardTicks++;

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

      rightRevs = rightForwardTicks / COUNTS_PER_REV;
      Serial.print("RIGHT: ");
      Serial.println(rightForwardTicks);
  }
}

  // Set up the external interrupt pins INT0 and INT1
  // for falling edge triggered. Use bare-metal.
  void setupEINT()
  {
    // Use bare-metal to configure pins 2 and 3 to be
    // falling edge triggered. Remember to enable
    // the INT0 and INT1 interrupts.

    EICRA = 0b00001010;
    EIMSK = 0b00000011;
  }

  // Implement the external interrupt ISRs below.
  // INT0 ISR should call leftISR while INT1 ISR
  // should call rightISR.

  ISR(INT0_vect)
  {
    leftISR();
  }

  ISR(INT1_vect)
  {
    rightISR();
  }


  // Implement INT0 and INT1 ISRs above.

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
    dir = FORWARD;

    int val = pwmVal(speed);

    // For now we will ignore dist and move
    // forward indefinitely. We will fix this
    // in Week 9.
    

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.

    
    clearCounters();
    while (forwardDist < dist)
    {
      analogWrite(LF, val);
      analogWrite(RF, val);
      analogWrite(LR, 0);
      analogWrite(RR, 0);
    }
    stop();
    clearCounters();
    
  }

  // Reverse Alex "dist" cm at speed "speed".
  // "speed" is expressed as a percentage. E.g. 50 is
  // reverse at half speed.
  // Specifying a distance of 0 means Alex will
  // continue reversing indefinitely.
  void reverse(float dist, float speed)
  {
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

  // Turn Alex left "ang" degrees at speed "speed".
  // "speed" is expressed as a percentage. E.g. 50 is
  // turn left at half speed.
  // Specifying an angle of 0 degrees will cause Alex to
  // turn left indefinitely.
  void left(float ang, float speed)
  {
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
  // turn right indefinitely.
  void right(float ang, float speed)
  {
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
    clearCounters();
    //  switch(which)
    //  {
    //    case 0:
    //      clearCounters();
    //      break;
    //
    //    case 1:
    //s      leftTicks=0;
    //      break;
    //
    //    case 2:
    //      rightTicks=0;
    //      break;
    //
    //    case 3:
    //      leftRevs=0;
    //      break;
    //
    //    case 4:
    //      rightRevs=0;
    //      break;
    //
    //    case 5:
    //      forwardDist=0;
    //      break;
    //
    //    case 6:
    //      reverseDist=0;
    //      break;
    //  }
  }
  // Intialize Vincet's internal states

  void initializeState()
  {
    clearCounters();
  }

  void handleCommand(TPacket * command)
  {
    switch (command->command)
    {
      // For movement commands, param[0] = distance, param[1] = speed.
      case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
        break;

      /*
         Implement code for other commands here.

      */

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

ISR(ADC_vect){ //Interrupt triggered upon completion of analog to digital conversion
    unsigned int loval = ADCL;
    unsigned int hival = ADCH;
    unsigned int adcvalue = hival * 256 + loval; // ADC conversion result is 10 bits, stored in ADCH & ADCL

  // Multiplex between the sensors
  // IR sensor is approximately digital (0 when too close)
  switch(ADMUX){ 
    case 0b01000000: // A0 
      adcvalue < 500 && !(overrideIR) ? Serial.println("Left collision detect"): 1;
      ADMUX = 0b01000001;
      break;
    
    case 0b01000001: // A1
      adcvalue < 500 && !(overrideIR) ? Serial.println("Centre collision detect"): 1;
      ADMUX = 0b01000010;
      break;

    case 0b01000010: // A2
      adcvalue < 500 && !(overrideIR) ? Serial.println("Right collision detect"): 1;
      ADMUX = 0b01000000;
      break;
  }
  ADCSRA |= 0b01000000; // Restart ADC conversion
}

void setupColourSensor(){
  // Set S0:4, S3:7, S1:3, S2:9 as output
  // Set sensorOut:8 as input
  DDRD |= 0b10010000;
  DDRB |= 0b00110000;
  DDRB &= 0b11111110; 
  
  //power saver mode - set S0 and S1 as LOW
  PORTD &= 0b11100111;
}

int colourDetect(){
  int sensorOut = 8;
  int red, blue, green;

  // Set at 20% power - Set S0 as HIGH and S1 as LOW
  PORTD |= 0b00010000;
  PORTD &= 0b11110111;
  delay(300);
  Serial.println("hi");
  
  //Detect red colour - Set S2 and S3 as LOW
  DDRB &= 0b11111101;
  DDRD &= 0b01111111;
  //delay(300);
  red = pulseIn(sensorOut, LOW);
  
  //Detect green colour - Set S2 and S3 as HIGH
  DDRB |= 0b00000010;
  DDRD |= 0b00010000;
  //delay(300);
  green = pulseIn(sensorOut, LOW);

  //Detect green colour - Set S2 as low and S3 as HIGH
  DDRB &= 0b11111101;
  DDRD |= 0b00010000;
  //delay(300);
  blue = pulseIn(sensorOut, LOW);

  Serial.print("Red = ");
  Serial.println(red);

  
  Serial.print("Green = ");
  Serial.println(green);

  
  Serial.print("Blue = ");
  Serial.println(blue);
  
  Serial.println((red < 550 && blue > red && green > red));

  //power saver mode
  PORTD &= 0b11101111;
  PORTB &= 0b11101111; 
  
  return (red < 550 && blue > red && green > red);
}

void setup() {
    // put your setup code here, to run once:

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

    // forward(0, 100);

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
      delay(100);
      colourDetect();

  }
