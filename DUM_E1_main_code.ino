// =======================================================================================
//        DUM-E1 Code
// =======================================================================================
//                          Last Revised Date: 12/27/2023
//                      Developed By: Michael Steckel (steckasaurus)
// =======================================================================================
//
// This code should serve as an example of the fact that I have no idea what the hell I 
// am doing. I just steal code snippets from other peoeple and hope that they work. I have
// tried to add comments indicating who or where I stole sections from. I have left the 
// comments by the original code writers where appropriate.
//
// Here is the goal: using two BLDC motors with encoders, an Arduino, a FrSky transmitter, 
// and a mechanical setup of DUM-E1's left arm, wave Dummy's hand up/down and left/right 
// via the transmitter controls.
//
// My setup at the moment: I am using one Tinymovr servo R5.2 (motor complete with driver and encoder) 
// and one GM3506 gimbal motor fitted with a Tinymovr M5.1 motor controller board. From the perspective of 
// Tinymovr and Arduino code, these two motors should control the same. These motors talk via CANbus. 
// I'm using an MKR WIFI 1010 board with an MKR CAN shield. I chose this board and shield because
// Tinymovr had a sketch for the 1010. I am using a FrSky TD18 receiver sending SBUS signal 
// to the Arduino via the serial port. 
// 
// I got the Tinymovr Arduino library from here: https://github.com/tinymovr/Tinymovr-arduino
// That github also has the MKR 1010 example code, which I stole a lot of for this sketch.
//
// The SBUS signal needs to be decoded so it can be used for further programming. I purchased 
// a signal inverter board (thanks Neil) to invert the incoming signal to the Arduino. And I
// took the code for importing the SBUS from Robotmaker at: https://github.com/robotmaker/Arduino_SBUS
//
// =======================================================================================
//   
// ---------------------------------------------------------------------------------------
//                          Libraries
// ---------------------------------------------------------------------------------------

/* This section is from the 1010 sketch
  !!! USE ONLY THE "CAN Adafruit Fork Library"             !!!
   !!! IT IS THE ONLY ONE WITH PROPER FILTER IMPLEMENTATION !!! */

#include "Arduino.h"
#include <CAN.h>  // "CAN Adafruit Fork" library
#include <tinymovr.hpp>

// I don't know if I have to define the serial port pins, but Robotmaker did it, so I did it.
#define rxPin 13
#define txPin 14

// ---------------------------------------------------------------------------------------
//                          Local Includes
// ---------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------
//               SYSTEM VARIABLES
// ---------------------------------------------------------------------------------------

// =======================================================================================
//                          Main Program
// =======================================================================================

// =======================================================================================
//                          Initialize - Setup Function
// =======================================================================================

// The Tinymovr object per the 1010 sketch
Tinymovr tinymovr(1, &send_cb, &recv_cb, &delay_us_cb, 100);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // start the CAN bus at 1Mbps
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // NOTE: You NEED to enable filtering using this pattern,
  // otherwise the library will not function correctly,
  // especially with a lot of Tinymovr units on the bus
  if (!CAN.filterExtended(0x0, 0x700))
  {
    Serial.println("Setting CAN filters failed!");
    while (1);
  }

// This part from Robotmaker
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    Serial.begin(100000,SERIAL_8E2);

}

// =======================================================================================
//           Main Program Loop - This is the recurring check loop for entire sketch
// =======================================================================================

void loop() {

// From this HERE HERE HERE HERE HERE HERE HERE HERE down to the next HERE is all from 
// Robotmaker for decoding the SBUS signal

  static byte          buffer[25];
  static int           channels[18];
  static int           errors = 0;
  static bool          failsafe = 0;
  static int           idx;
  static unsigned long last_refresh = 0;
  static int           lost = 0;
  byte b;
  int  i;

  //Check the serial port for incoming data
 //This could also be done via the serialEvent()
  if (Serial.available ()) {
      b = Serial.read ();
       
     //this is a new package and it' not zero byte then it's probably the start byte B11110000 (sent MSB)
     //so start reading the 25 byte package
      if (idx == 0 && b != 0x0F) {  // start byte 15?
       // error - wait for the start byte
        
      } else {
        buffer[idx++] = b;  // fill the buffer with the bytes until the end byte B0000000 is recived
      }
   
    if (idx == 25) {  // If we've got 25 bytes then this is a good package so start to decode
      idx = 0;
      if (buffer[24] != 0x00) {
        errors++;
      } else 
      {
            //  Serial.println("Found Packet");
            // 25 byte packet received is little endian. Details of how the package is explained on our website:
            //http://www.robotmaker.eu/ROBOTmaker/quadcopter-3d-proximity-sensing/sbus-graphical-representation
            channels[1]  = ((buffer[1]    |buffer[2]<<8)                 & 0x07FF);
            channels[2]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
            channels[3]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF);
            channels[4]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
            channels[5]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
            channels[6]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
            channels[7]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
            channels[8]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
            channels[9]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
            channels[10]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
            channels[11] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
            channels[12] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
            channels[13] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
            channels[14] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
            channels[15] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
            channels[16] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);
            channels[17] = ((buffer[23])      & 0x0001) ? 2047 : 0;
            channels[18] = ((buffer[23] >> 1) & 0x0001) ? 2047 : 0;
     
            failsafe = ((buffer[23] >> 3) & 0x0001) ? 1 : 0;
            if ((buffer[23] >> 2) & 0x0001) lost++;

            // HERE HERE HERE HERE HERE HERE HERE HERE ends the Robotmaker decoding section

            //***********************************************************************
            //The channels used are 5 and 6. Mapped these from their 0-2000 range to -4000 to 4000 tinymovr output range
            // which should correspond to almost minus half a turn to plus half a turn.
            // Thanks Steve Foster for the tip on mapping. It turns out that the Robotmaker sketch also 
            // used mapping to map channels to change the RGB color values on an LED. So I 
            // modeled my map after that code
            //***********************************************************************
            int UpDownPosition = map(channels[5],0,2000,-4000,4000);
            int LeftRightPosition = map(channels[6],0,2000,-4000,4000);

            //once mapped, the motors just need to be told to move to the new mapped positions. See below.
  
    // The Tinymovr 1010 sketch is set up to read the serial interface and then do a handful of different
    // commands based on the input. I know that won't work for what I want to do but I left these
    // code snippets here for reference. 
    //
    //  Serial.println("Received L turn command");
    //  float pos_estimate = tinymovr.encoder.get_position_estimate();
    //  Serial.println(pos_estimate);
    //  tinymovr.controller.position.set_setpoint(pos_estimate - 8192.0f);
    
    //  Serial.println("Received R turn command");
    //  float pos_estimate = tinymovr.encoder.get_position_estimate();
    //  Serial.println(pos_estimate);
    //  tinymovr.controller.position.set_setpoint(pos_estimate + 8192.0f);
    //
    // Seems to me I can just use the set_setpoint command, replacing the "pos_estimate + 8192.0f)" with 
    // either UpDownPosition or LeftRightPosition. But the part I can't figure out is 
    // how to indicate to the two different Tinymovr's which command is for up/down and which for
    // left/right.
    
        } //closing - else
      } //closing - if (idx == 25)
    } //closing - if (Serial.available ())
} //closing void loop

// =======================================================================================
//           Subroutines - These are the different functions called by the main loop
// They are also all from the Tinymovr 1010 sketch. I believe they are utilized mostly by
// the Tinymovr Arduino library but I'm not sure.
// =======================================================================================

/*
 * Function:  send_cb 
 * --------------------
 *  Is called to send a CAN frame
 *
 *  arbitration_id: the frame arbitration id
 *  data: pointer to the data array to be transmitted
 *  data_size: the size of transmitted data
 *  rtr: if the frame is of request transmit type (RTR)
 */
void send_cb(uint32_t arbitration_id, uint8_t *data, uint8_t data_size, bool rtr)
{
  CAN.beginExtendedPacket(arbitration_id, data_size, rtr);
  for (int i=0; i<data_size; i++)
  {
    CAN.write(data[i]);
  }
  CAN.endPacket();
}

/*
 * Function:  recv_cb 
 * --------------------
 *  Is called to receive a CAN frame
 *
 *  arbitration_id: the frame arbitration id
 *  data: pointer to the data array to be received
 *  data_size: pointer to the variable that will hold the size of received data
 */
bool recv_cb(uint32_t *arbitration_id, uint8_t *data, uint8_t *data_size)
{
  int packetSize = CAN.parsePacket();
  if (packetSize > 0) {
    *data_size = packetSize;
    for (int i = 0; i < packetSize; i++) {
      int r = CAN.read();
      if (r == -1) return false;
      data[i] = (uint8_t)r;
    }
    *arbitration_id = CAN.packetId();
    return true;
  }
  return false;
}

/*
 * Function:  delay_us_cb 
 * --------------------
 *  Is called to perform a delay
 *
 *  us: the microseconds to wait for
 */
void delay_us_cb(uint32_t us)
{
  delayMicroseconds(us);
}
