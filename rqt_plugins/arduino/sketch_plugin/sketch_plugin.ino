#include <DynamixelSDK.h>

#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <rqt_plugins/Scan.h>
#include <rqt_plugins/PresentPosition.h>
#include <rqt_plugins/GoalTorque.h>

// Control table address (MX-106)
#define ADDR_PRO_TORQUE_ENABLE          24                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          30
#define ADDR_PRO_GOAL_TORQUE            71
#define ADDR_PRO_PRESENT_POSITION       36
#define ADDR_PRO_CONTROL_TORQUE_MODE    70
#define ADDR_PRO_MAX_TORQUE             14
#define ADDR_PRO_TORQUE_LIMIT           34
#define ADDR_PRO_CURRENT                68
#define ADDR_PRO_PRESENT_POSITION       36
#define ADDR_PRO_MODEL_NUMBER           0
#define ADDR_PRO_BAUD_RATE              4


// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1 by default
#define CURRENT_BAUDRATE                115200      // 57600
#define DEVICENAME                      "3"                 // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define TORQUE_CONTROL_ENABLE         1
#define TORQUE_CONTROL_DISABLE        0

#define BAUD_RATE_115200              16

ros::NodeHandle  nh;
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

void messageGoalTorqueCb( const rqt_plugins::GoalTorque& torque_msg) {

    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0; 
    
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, torque_msg.id, ADDR_PRO_GOAL_TORQUE, torque_msg.goalTorque, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

}

// Create a service server that takes nothing and returns nothing
void callbackServiceScan(const rqt_plugins::Scan::Request & req, rqt_plugins::Scan::Response & res)
{
  //nh.loginfo("Callback Service Scan!");
  uint8_t dxl_error = 0;
  uint16_t dxl_model_number = 0;
  const char *dxl_string_error, *dxl_string_comm_result;

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
     
  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, req.id, ADDR_PRO_MODEL_NUMBER, &dxl_model_number, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
      res.modelNumber = 0;
      res.dxl_comm_result = dxl_comm_result;
      dxl_string_comm_result = packetHandler->getTxRxResult(dxl_comm_result);
      res.dxl_string_comm_result.data = dxl_string_comm_result;
      
  }
  else if (dxl_error != 0)
  {
    res.modelNumber = 0;
    res.dxl_error = dxl_error;
    dxl_string_error = packetHandler->getRxPacketError(dxl_error);
    res.dxl_string_error.data = dxl_string_error;
    
  }
  else
  {
    res.modelNumber = dxl_model_number;
    res.dxl_string_comm_result.data = "";
    res.dxl_string_error.data = "";
  }

}

rqt_plugins::PresentPosition present_position_msg;
ros::ServiceServer<rqt_plugins::Scan::Request, rqt_plugins::Scan::Response> serverScan("scan",&callbackServiceScan);
ros::Publisher pub_present_position("PresentPosition", &present_position_msg);
ros::Subscriber<rqt_plugins::GoalTorque> subGoalTorque("GoalTorque", messageGoalTorqueCb );


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("Start..");

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error

  
  nh.initNode();
  nh.advertiseService(serverScan);
  nh.advertise(pub_present_position);
  nh.subscribe(subGoalTorque);


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

   /*delay(100);*/
   
  const char *dxl_string_error, *dxl_string_comm_result;

  // Change baudrate
 /* dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_BAUD_RATE, BAUD_RATE_115200, &dxl_error);
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


  delay(100);
*/
  // Read current baudrate
  uint8_t dxl_current_baud_rate = 0; 
  dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_BAUD_RATE, (uint8_t*)&dxl_current_baud_rate, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
    dxl_string_comm_result = packetHandler->getTxRxResult(dxl_comm_result);
    Serial.println("Pb of communication to get baud rate ! \n");
    Serial.println(dxl_string_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
    dxl_string_error = packetHandler->getRxPacketError(dxl_error);
    Serial.println("Error to get baud rate ! \n");
    Serial.println(dxl_string_error);
  }
  else
  {
    Serial.print("Ok read current baud rate ! \n");
    Serial.println("[Baud Rate : ");      
    Serial.print(dxl_current_baud_rate);
    Serial.print(" ]");
    Serial.println("");
    }
  
 // Enable Dynamixel Control Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_CONTROL_TORQUE_MODE, TORQUE_CONTROL_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  else
  {
    Serial.print("Dynamixel -> successfully Enable Dynamixel Control Torque ! \n");}

}

void loop() {
  // put your main code here, to run repeatedly:

  uint8_t dxl_error = 0;
  uint16_t dxl_present_position;    
  packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, &dxl_present_position, &dxl_error);
  present_position_msg.presentPosition = dxl_present_position;
  present_position_msg.id = DXL_ID;

  //nh.loginfo("Publish Present Position !");
  pub_present_position.publish(&present_position_msg);

  
  nh.spinOnce();

  //delay(100);
}
