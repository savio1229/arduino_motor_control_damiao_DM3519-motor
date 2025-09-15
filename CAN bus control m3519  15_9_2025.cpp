#include <Arduino.h>


//for can bus
#include <SPI.h>
#include <mcp2515.h>

// Define CAN Bus Related Stuff
struct can_frame canMsgIn;   // CAN frame for incoming messages
struct can_frame canMsgOut;  // CAN frame for outgoing messages
MCP2515 mcp2515(10);         // MCP2515 CAN bus controller instance

/*
  Connection Diagram:
  MCP2515 Pin  |  Arduino Mega 2560 Pin
  -------------|-----------------------
  VCC          |  5V
  GND          |  GND
  CS           |  Pin 53 (SS)
  SO           |  Pin 50 (MISO)
  SI           |  Pin 51 (MOSI)
  SCK          |  Pin 52 (SCK)
  INT          |  Pin 2

*/
/*
  Connection Diagram:
  MCP2515 Pin  |  Arduino uno Pin
  -------------|-----------------------
  VCC          |  5V
  GND          |  GND
  CS           |  Pin 10 (SS)
  SO           |  Pin 12 (MISO)
  SI           |  Pin 11 (MOSI)
  SCK          |  Pin 13 (SCK)
  INT          |  Pin 2
*/

// Global variable to store the output message
String output_message;

void can_mcp2515_setup() {
  mcp2515.reset();                             // Reset MCP2515 controller
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);  // Set CAN bus bitrate to 1 Mbps and clock frequency to 8 MHz
  mcp2515.setNormalMode();                     // Set MCP2515 to normal mode
}



//for m3508 Vel_desired
float Vel_desired = 12.34;  // example value
uint8_t data_buf[8];        // make sure the buffer is large enough


void can_write_move_motor_DM3519(int motor_ID, int rpm){

  canMsgOut.can_id = 0x200 + motor_ID;  // Set CAN message ID

  int rad(rpm*3.14159265359/30);
  float Vel_desire = rad;
  Serial.print("DM3519rpm: ");
  Serial.print(rpm);
union {
  float value;
  uint8_t bytes[4];
} floatToBytes;

floatToBytes.value = Vel_desired;

// copy to data_buf[0..3]
for (int i = 0; i < 4; i++) {
  data_buf[i] = floatToBytes.bytes[i];
  canMsgOut.data[i] = floatToBytes.bytes[i];
  }
}

void setup() {
  // put your setup code here, to run once:
 can_mcp2515_setup();
 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

}

