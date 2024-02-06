// =======================================================================================
//        DUM-E1 Main Code
// =======================================================================================
//                          Last Revised Date: 02/04/2024
//                      Developed By: Michael Steckel (steckasaurus)
// =======================================================================================
//
// Here is the goal: using two BLDC motors with encoders, an Arduino, a FrSky transmitter, 
// and a mechanical setup of DUM-E1's left arm, wave Dummy's hand up/down and left/right 
// via the transmitter controls.
//
// My setup at the moment: I am using one Tinymovr servo R5.2 (motor complete with driver and encoder) 
// and one GM3506 gimbal motor fitted with a Tinymovr M5.1 motor controller board. From the perspective of 
// Tinymovr and Arduino code, these two motors should control the same. These motors talk via CANbus. 
// I'm using an MKR WIFI 1010 board with an MKR CAN shield. I chose this board and shield because
// Tinymovr had a sketch for the 1010. I am using a FrSky TD R18 receiver sending SBUS signal 
// to the Arduino via the serial port. 
// 
// I got the Tinymovr Arduino library from here: https://github.com/tinymovr/Tinymovr-arduino
// That github also has the MKR 1010 example code, which I stole a lot of for this sketch.
//
// The SBUS signal needs to be decoded so it can be used for further programming. I purchased 
// a signal inverter board (thanks Neil) to invert the incoming signal to the Arduino. And I
// took the code for importing the SBUS from Steve Foster in the R2 Builder's Group 
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

//This section is for the SBUS decode
#include "sbus.h"
#include "wiring_private.h"
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
extern Stream *rcSerialPort;
bfs::SbusRx sbus_rx(&Serial3);
bfs::SbusData data;

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
//           Subroutines - These are the different functions called by the main loop
// The first three are from the Tinymovr 1010 sketch. I believe they are utilized mostly by
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
// =======================================================================================
//                          Initialize - Setup Function
// =======================================================================================

// The Tinymovr object per the 1010 sketch
Tinymovr tinymovr(1, &send_cb, &recv_cb, &delay_us_cb, 100);

void setup() {

  //SBUS decode section
  Serial.begin(115200);
  while (!Serial) {}
  delay(2000);
  pinPeripheral(1, PIO_SERCOM);  // Assign RX function to pin 1
  Serial.print("Begin SBUS...");
  sbus_rx.Begin();
  Serial.println("ok");

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

}

// =======================================================================================
//           Main Program Loop - This is the recurring check loop for entire sketch
// =======================================================================================

void loop() {

if (sbus_rx.Read()) {
    data = sbus_rx.data();
    //for (int8_t i = 0; i < data.NUM_CH; i++) {
    //  Serial.print(data.ch[i]);
    //  Serial.print("\t");
    float UpDownPosition = map(data.ch[4],0,2000,-4000.0f,4000.0f);
    float LeftRightPosition = map(data.ch[5],0,2000,-4000.0f,4000.0f);
    Serial.print(UpDownPosition);
    Serial.print("\t");
    Serial.print(LeftRightPosition);
    Serial.print("\t");
    // Hopefully commanding the motor to move to the position indicated by the transmitter/receiver in this next line.
    // Testing just one motor at the moment, so randomly chose UpDown 
    tinymovr.controller.position.set_setpoint(UpDownPosition);
    }
    Serial.print(data.lost_frame);
    Serial.print("\t");
    Serial.println(data.failsafe);
  

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
    
//Setting tinymovr to closed loop position control (from the 1010 sketch)
tinymovr.controller.set_state(2);
tinymovr.controller.set_mode(2);

} //closing void loop




//This sub used by the SBUS decode
void SERCOM3_Handler() {
  Serial3.IrqHandler();
}