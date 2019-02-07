#include <DynamixelSDK.h>

#define ADDR_PRO_BAUD_RATE              4
// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1 by default
#define CURRENT_BAUDRATE                115200            // 57600
#define DEVICENAME                      "3"                 // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define NEW_BAUD_RATE_115200              16
#define NEW_BAUD_RATE_1000000             1
#define NEW_BAUD_RATE_57600               34

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("Start..");

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
   portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);


  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  const char *dxl_string_error, *dxl_string_comm_result;

  int current_baud_rate = portHandler->getBaudRate();
  Serial.println("[Current baudrate = ");
  Serial.print(current_baud_rate);
  Serial.println(" ]");
  
  
  // Set port baudrate
  if (portHandler->setBaudRate(CURRENT_BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

 
  // Change baudrate
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_BAUD_RATE, NEW_BAUD_RATE_57600, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
    dxl_string_comm_result = packetHandler->getTxRxResult(dxl_comm_result);
    Serial.println("Pb of communication to set baud rate ! \n");
    Serial.println(dxl_string_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
    dxl_string_error = packetHandler->getRxPacketError(dxl_error);
    Serial.println("Error to set baud rate ! \n");
    Serial.println(dxl_string_error);
  }
  else
  {
    Serial.print("Dynamixel has been successfully change the baudrate ! \n");}



  // Close port
  Serial.println("Close Port ! \n");
  portHandler->closePort();

  Serial.println("Finish ! \n");

}

void loop() {
  // put your main code here, to run repeatedly:

}
