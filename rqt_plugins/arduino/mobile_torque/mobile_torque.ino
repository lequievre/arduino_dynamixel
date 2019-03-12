
/*
 * Laurent LEQUIEVRE
 * Research Engineer, CNRS (France)
 * laurent.lequievre@uca.fr
 * UMR 6602 - Institut Pascal
 * 
 * rostopic pub -1 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
 * rostopic echo /joint_states
 * 
 */

#include <DynamixelWorkbench.h>
#include <math.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

// For Matrix
#include <BasicLinearAlgebra.h>

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT 1

// Protocol 2.0
#define ADDR_PRESENT_CURRENT_2    126
#define ADDR_PRESENT_VELOCITY_2   128
#define ADDR_PRESENT_POSITION_2   132

#define ADDR_GOAL_CURRENT_2   102
#define LENGTH_GOAL_CURRENT_2  2

#define LENGTH_PRESENT_CURRENT_2  2
#define LENGTH_PRESENT_VELOCITY_2 4
#define LENGTH_PRESENT_POSITION_2 4




#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif  

#define BAUDRATE              57600
#define DXL_ID_WHEEL_RIGHT    2
#define DXL_ID_WHEEL_LEFT     3
#define NB_WHEEL              2

#define CURRENT_UNIT          3.36f     // 3.36[mA]
#define RPM_MX_64_2           0.229     // 0.229 rpm

#define WHEEL_RADIUS          0.047     // 4.7 cm
#define WHEEL_SEPARATION      0.21      // 21 cm
#define WHEEL_MASS            0.240     // 0.24 kg
#define WHEEL_INERTIA         (0.5 * WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS)

#define LINK_MASS             0.168     // 0.168 kg (platic link + motor)
#define ARM_LINK_MASS         (3 * LINK_MASS)  // mass of arm based on 3 links

#define BASE_HEIGHT           0.16      // 0.16 m
#define BASE_WIDTH            0.145     // 0.145 m
#define BASE_MASS             1.488     // 1.488 kg
#define BASE_INERTIA          ((1/12) * BASE_MASS * ((BASE_HEIGHT * BASE_HEIGHT) + (BASE_WIDTH * BASE_WIDTH)))

#define DEVICE_MASS           (BASE_MASS + ARM_LINK_MASS + (2 * WHEEL_MASS))

#define GAIN_KV1              1.0
#define GAIN_KV2              1.0

#define CURRENT_LIMIT         1941  // max current (no unit defined) 1941
#define MAX_TORQUE            6   // max torque 6Nm
#define TORQUE_TO_CURRENT     (CURRENT_LIMIT/MAX_TORQUE)

DynamixelWorkbench dxl_wb;

union myUnion {
  uint32_t my4byteNumber;
  uint16_t my2byteNumber[2];
};


union myUnionForCurrent {
  int32_t my4byteNumber;
  int16_t my2byteNumber[2];
};

ros::NodeHandle  nh;
sensor_msgs::JointState joint_states_msg;
ros::Publisher joint_states_pub("joint_states", &joint_states_msg);

// Use publisher to debug matrix results !
std_msgs::Float64MultiArray debug_matrix_msg;
ros::Publisher debug_matrix_pub("debug_matrix", &debug_matrix_msg);

std::map<std::string, uint8_t> map_id_wheel_dynamixels;

uint8_t wheel_id_array[2] = {DXL_ID_WHEEL_RIGHT, DXL_ID_WHEEL_LEFT};

int32_t wheel_raw_position[2] = {0, 0};
float wheel_position[2] = { 0.0, 0.0 };
int32_t wheel_raw_current[2] = {0, 0};
float wheel_current[2] = { 0.0, 0.0 };
int32_t wheel_raw_velocity[2] = {0, 0};
float wheel_velocity[2] = { 0.0, 0.0 };
char *wheel_names[2] = {"wheel_right", "wheel_left"};

using namespace BLA;

ArrayMatrix<2,3,double> matrix_Jinv;
ArrayMatrix<2,1,double> matrix_wheelAngularVelocityCommand;
ArrayMatrix<2,1,double> matrix_wheelAngularAcc;
ArrayMatrix<2,2,double> matrix_wheelVelocityGain = {GAIN_KV1, 0, 0, GAIN_KV2};

ArrayMatrix<2,2,double> matrix_inertia = { 
((DEVICE_MASS * WHEEL_RADIUS * WHEEL_RADIUS * 0.25) + WHEEL_INERTIA + ( ((WHEEL_RADIUS * WHEEL_RADIUS)/(WHEEL_SEPARATION *WHEEL_SEPARATION)) * BASE_INERTIA )),
((DEVICE_MASS * WHEEL_RADIUS * WHEEL_RADIUS * 0.25) - ( ((WHEEL_RADIUS * WHEEL_RADIUS)/(WHEEL_SEPARATION *WHEEL_SEPARATION)) * BASE_INERTIA )),
((DEVICE_MASS * WHEEL_RADIUS * WHEEL_RADIUS * 0.25) - ( ((WHEEL_RADIUS * WHEEL_RADIUS)/(WHEEL_SEPARATION *WHEEL_SEPARATION)) * BASE_INERTIA )),
((DEVICE_MASS * WHEEL_RADIUS * WHEEL_RADIUS * 0.25) + WHEEL_INERTIA + ( ((WHEEL_RADIUS * WHEEL_RADIUS)/(WHEEL_SEPARATION *WHEEL_SEPARATION)) * BASE_INERTIA ))
};

ArrayMatrix<3,1,double> matrix_wheelVelocityCommand = {0.0, 0.0, 0.0};
ArrayMatrix<2,1,double> matrix_wheelVelocityResponse = {0.0, 0.0};
ArrayMatrix<2,1,double> matrix_torque = {0.0, 0.0};

double cmd_linear_x = 0.0;
double cmd_linear_y = 0.0;
double cmd_angular_z = 0.0;

void updateJinv(ArrayMatrix<2,3,double> & J, double orientation, double radius, double wheel_separation)
{

  J <<  cos(orientation)/radius, sin(orientation)/radius, wheel_separation/(2*radius), 
        cos(orientation)/radius, sin(orientation)/radius, -wheel_separation/(2*radius);
}

bool initWheelSyncWrite(void)
{
  bool result = false;
  const char* log;
  
  result = dxl_wb.addSyncWriteHandler(map_id_wheel_dynamixels["wheel_right"], "Goal_Current", &log);
  //result = dxl_wb.addSyncWriteHandler(ADDR_GOAL_CURRENT_2, LENGTH_GOAL_CURRENT_2, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler goal current wheel right");
    return false;
  }
  else
  {
    Serial.println("Succeeded to add sync write handler goal current wheel right");
  }
  
  result = dxl_wb.addSyncWriteHandler(map_id_wheel_dynamixels["wheel_left"], "Goal_Current", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler goal current wheel left");
    return false;
  }
  else
  {
    Serial.println("Succeeded to add sync write handler goal current wheel left");
  }

  return true;
}

bool initWheelSyncRead(void)
{
  bool result = false;
  const char* log;

  result = dxl_wb.addSyncReadHandler(ADDR_PRESENT_CURRENT_2,
                                          (LENGTH_PRESENT_CURRENT_2 + LENGTH_PRESENT_VELOCITY_2 + LENGTH_PRESENT_POSITION_2),
                                          &log);

  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync read handler present position + present velocity + present current");
    return false;
  }
  else
  {
    Serial.println("Succeeded to add sync read handler present position + present velocity + present current");
  }

  return true;                         
}

bool readWheelSyncDatas()
{
    bool result = false;
    const char* log;
    
    myUnion unionOfBytes;
    int16_t currentRaw = 0;
    

    wheel_raw_current[0] = 0;
    wheel_raw_current[1] = 0;

    result = dxl_wb.syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                               wheel_id_array,
                               NB_WHEEL,
                               &log);

   result = dxl_wb.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                          wheel_id_array,
                          NB_WHEEL,
                          ADDR_PRESENT_CURRENT_2,
                          LENGTH_PRESENT_CURRENT_2,
                          wheel_raw_current,
                          &log);


   unionOfBytes.my4byteNumber = wheel_raw_current[0];
   currentRaw = unionOfBytes.my2byteNumber[0];
   wheel_current[0] = currentRaw * CURRENT_UNIT;

   unionOfBytes.my4byteNumber = wheel_raw_current[1];
   currentRaw = unionOfBytes.my2byteNumber[0];
   wheel_current[1] = currentRaw * CURRENT_UNIT;

         
   result = dxl_wb.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                      wheel_id_array,
                                                      NB_WHEEL,
                                                      ADDR_PRESENT_VELOCITY_2,
                                                      LENGTH_PRESENT_VELOCITY_2,
                                                      wheel_raw_velocity,
                                                      &log);


    wheel_velocity[0] = dxl_wb.convertValue2Velocity(wheel_id_array[0], wheel_raw_velocity[0]);
    wheel_velocity[1] = dxl_wb.convertValue2Velocity(wheel_id_array[1], wheel_raw_velocity[1]);
    
  
    result = dxl_wb.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                    wheel_id_array,
                                                    NB_WHEEL,
                                                    ADDR_PRESENT_POSITION_2,
                                                    LENGTH_PRESENT_POSITION_2,
                                                    wheel_raw_position,
                                                    &log);

    wheel_position[0] = fmod(dxl_wb.convertValue2Radian(wheel_id_array[0], wheel_raw_position[0]),2.0*M_PI); 
    wheel_position[1] = fmod(dxl_wb.convertValue2Radian(wheel_id_array[1], wheel_raw_position[1]),2.0*M_PI);

}

void commandVelocityCallback(const geometry_msgs::Twist &msg)
{
  //bool result = false;
  //const char* log;

  cmd_linear_x = msg.linear.x;
  cmd_linear_y = msg.linear.y;
  cmd_angular_z = msg.angular.z;

  
  matrix_wheelVelocityCommand << cmd_linear_x, cmd_linear_y, cmd_angular_z;
  
  //matrix_wheelVelocityCommand(0) = cmd_linear_x;
  //matrix_wheelVelocityCommand(1) = cmd_linear_y;
  //matrix_wheelVelocityCommand(2) = cmd_angular_z;
  
  /*int32_t current_limit = 0;
  result = dxl_wb.readRegister(DXL_ID_WHEEL_RIGHT, "Current_Limit", &current_limit, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read current limit !");
  }
  else
  {
    Serial.print("Current limit : "); Serial.println(current_limit);
  }*/

  
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback );

bool initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  result = dxl_wb.init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init workbench !");
    return false;
  }
  else
  {
    Serial.print("Succeeded to init workbench with a baudrate : ");
    Serial.println(BAUDRATE);
  }

  return true;
}

void getDynamixelsWheelInfo()
{
  map_id_wheel_dynamixels["wheel_right"] = DXL_ID_WHEEL_RIGHT;
  map_id_wheel_dynamixels["wheel_left"] = DXL_ID_WHEEL_LEFT;
}

bool loadWheelDynamixels(void)
{
  bool result = false;
  const char* log;
  uint16_t model_number = 0;

  std::map<std::string, uint8_t>::iterator it = map_id_wheel_dynamixels.begin();

  while (it != map_id_wheel_dynamixels.end())
  {
    result = dxl_wb.ping((uint8_t)it->second, &model_number, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.print("Failed to ping ");
      Serial.print((it->first).c_str());
      Serial.println(" !");
      Serial.print("ID : ");
      Serial.println((uint8_t)it->second);
      return false;
    }
    else
    {
      Serial.println("Succeeded to ping : ");
      Serial.print("name : ");
      Serial.println((it->first).c_str());
      Serial.print("id : ");
      Serial.println((uint8_t)it->second);
      Serial.print("model_number : ");
      Serial.println(model_number);
      Serial.print("model name : ");
      Serial.println(dxl_wb.getModelName((uint8_t)it->second));
    }
    it++;
  }
  
  return true;
}

bool initWheelDynamixels(void)
{
  bool result = false;
  const char* log;
  uint16_t model_number = 0;

  std::map<std::string, uint8_t>::iterator it = map_id_wheel_dynamixels.begin();

  while (it != map_id_wheel_dynamixels.end())
  {
    result = dxl_wb.setCurrentControlMode((uint8_t)it->second, &log);
    //result = dxl_wb.setCurrentBasedPositionControlMode((uint8_t)it->second, &log);
    //result = dxl_wb.writeRegister((uint8_t)it->second, "Operating_Mode", 0, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.print("Failed to set the Current Control Mode !");
      Serial.print((it->first).c_str());
      Serial.println(" !");
      Serial.print("ID : ");
      Serial.println((uint8_t)it->second);
      return false;
    }
    else
    {
      Serial.println("Succeeded  to set the Current Control Mode !");
      Serial.print("id : ");
      Serial.println((uint8_t)it->second);
      Serial.print("model_number : ");
      Serial.println(model_number);
      Serial.print("model name : ");
      Serial.println(dxl_wb.getModelName((uint8_t)it->second));
    }

    // Set Goal Current to 0
    result = dxl_wb.writeRegister((uint8_t)it->second, "Goal_Current", 0, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.print("Failed to set the Goal Current to 0 !");
      Serial.print((it->first).c_str());
      Serial.println(" !");
      Serial.print("ID : ");
      Serial.println((uint8_t)it->second);
      return false;
    }
    else
    {
      Serial.println("Succeeded  to set the Current Current to 0 !");
      Serial.print((it->first).c_str());
      Serial.println(" !");
      Serial.print("ID : ");
      Serial.println((uint8_t)it->second);
    }

    // Set Torque to ON
    result = dxl_wb.torqueOn((uint8_t)it->second, &log);
    //result = dxl_wb.writeRegister((uint8_t)it->second, "Torque_Enable", 1, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.print("Failed to set Torque to ON !");
      Serial.print((it->first).c_str());
      Serial.println(" !");
      Serial.print("ID : ");
      Serial.println((uint8_t)it->second);
      return false;
    }
    else
    {
      Serial.println("Succeeded  to set Torque to ON !");
      Serial.print((it->first).c_str());
      Serial.println(" !");
      Serial.print("ID : ");
      Serial.println((uint8_t)it->second);
    }
    
    it++;
    
  }

 /*
  // Change Drive_Mode for Right Wheel
  int32_t drive_mode = 0;
  result = dxl_wb.writeRegister(DXL_ID_WHEEL_RIGHT, "Drive_Mode", drive_mode, &log);
  if (result == false)
  {
      Serial.println(log);
      Serial.print("Failed to change Drive Mode of the Right Wheel !");
  }
  else
  {
      Serial.println("Succeeded to change Drive Mode of the Right Wheel !");
  }*/

  return true;
}

void printWheelsInfos(void)
{
  int32_t velocity_limit = 0;
  int32_t current_limit = 0;
  int32_t  drive_mode = 0;
  int32_t  operating_mode = 0;
  int32_t  firmware_version = 0;
  bool result = false;
  const char* log;

  const ModelInfo* modelInfoWheelRight =  dxl_wb.getModelInfo(DXL_ID_WHEEL_RIGHT, &log);
  Serial.println("Wheel Right Infos :");
  Serial.print("rpm : "); Serial.println(modelInfoWheelRight->rpm);
  Serial.print("value_of_min_radian_position : "); Serial.println((int32_t)modelInfoWheelRight->value_of_min_radian_position);
  Serial.print("value_of_zero_radian_position : "); Serial.println((int32_t)modelInfoWheelRight->value_of_zero_radian_position);
  Serial.print("value_of_max_radian_position : "); Serial.println((int32_t)modelInfoWheelRight->value_of_max_radian_position);
  Serial.print("min_radian : "); Serial.println(modelInfoWheelRight->min_radian);
  Serial.print("max_radian : "); Serial.println(modelInfoWheelRight->max_radian);

  result = dxl_wb.readRegister(DXL_ID_WHEEL_RIGHT, "Velocity_Limit", &velocity_limit, &log);
  Serial.print("velocity limit : "); Serial.println(velocity_limit);
  result = dxl_wb.readRegister(DXL_ID_WHEEL_RIGHT, "Drive_Mode", &drive_mode, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read the Drive Mode !");
  }
  else
  {
    Serial.print("drive mode : "); Serial.println(drive_mode);
  }
  result = dxl_wb.readRegister(DXL_ID_WHEEL_RIGHT, "Operating_Mode", &operating_mode, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read the Operating Mode !");
  }
  else
  {
    Serial.print("operating mode : "); Serial.println(operating_mode);
  }
  result = dxl_wb.readRegister(DXL_ID_WHEEL_RIGHT, "Firmware_Version", &firmware_version, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read Firmware version !");
  }
  else
  {
    Serial.print("firmware version : "); Serial.println(firmware_version);
  }
  result = dxl_wb.readRegister(DXL_ID_WHEEL_RIGHT, "Current_Limit", &current_limit, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read current limit !");
  }
  else
  {
    Serial.print("Current limit : "); Serial.println(current_limit);
  }
  

  const ModelInfo* modelInfoWheelLeft =  dxl_wb.getModelInfo(DXL_ID_WHEEL_LEFT, &log);
  Serial.println("Wheel Left Infos :");
  Serial.print("rpm : "); Serial.println(modelInfoWheelLeft->rpm);
  Serial.print("value_of_min_radian_position : "); Serial.println((int32_t)modelInfoWheelLeft->value_of_min_radian_position);
  Serial.print("value_of_zero_radian_position : "); Serial.println((int32_t)modelInfoWheelLeft->value_of_zero_radian_position);
  Serial.print("value_of_max_radian_position : "); Serial.println((int32_t)modelInfoWheelLeft->value_of_max_radian_position);
  Serial.print("min_radian : "); Serial.println(modelInfoWheelLeft->min_radian);
  Serial.print("max_radian : "); Serial.println(modelInfoWheelLeft->max_radian);

  result = dxl_wb.readRegister(DXL_ID_WHEEL_LEFT, "Velocity_Limit", &velocity_limit, &log);
  Serial.println("");
  Serial.print("wheel left velocity limit : ");
  Serial.println(velocity_limit);
  result = dxl_wb.readRegister(DXL_ID_WHEEL_LEFT, "Drive_Mode", &drive_mode, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read the Drive Mode !");
  }
  else
  {
    Serial.print("drive mode : "); Serial.println(drive_mode);
  }
  result = dxl_wb.readRegister(DXL_ID_WHEEL_LEFT, "Operating_Mode", &operating_mode, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read the Operating Mode !");
  }
  else
  {
    Serial.print("operating mode : "); Serial.println(operating_mode);
  }
  result = dxl_wb.readRegister(DXL_ID_WHEEL_LEFT, "Firmware_Version", &firmware_version, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read Firmware version !");
  }
  else
  {
    Serial.print("firmware version : "); Serial.println(firmware_version);
  }
  result = dxl_wb.readRegister(DXL_ID_WHEEL_LEFT, "Current_Limit", &current_limit, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read current limit !");
  }
  else
  {
    Serial.print("Current limit : "); Serial.println(current_limit);
  }
  
}

void setup() {
  
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor
  
  nh.initNode();
  nh.advertise(joint_states_pub);
  nh.advertise(debug_matrix_pub);
  nh.subscribe(cmd_vel_sub);

  if (!initWorkbench(DEVICE_NAME,BAUDRATE)) return;
  getDynamixelsWheelInfo();
  if (!loadWheelDynamixels()) return;
  initWheelDynamixels();

  if (!initWheelSyncRead()) return;
  initWheelSyncWrite();

  printWheelsInfos();
}

unsigned long publisher_timer = 0;

void loop() {

  const char* log;
  
  readWheelSyncDatas();
  /*Serial.print("wheel right position = ");
  Serial.print(wheel_position[0]);
  Serial.print(" wheel left position = ");
  Serial.println(wheel_position[1]);
  Serial.println("wheel right velocity = ");
  Serial.print(wheel_velocity[0]);
  Serial.print(" wheel left velocity = ");
  Serial.println(wheel_velocity[1]);
  Serial.println("wheel right current = ");
  Serial.print(wheel_current[0]);
  Serial.print(" wheel left current = ");
  Serial.println(wheel_current[1]);*/

  matrix_Jinv <<  cos(cmd_angular_z)/WHEEL_RADIUS, sin(cmd_angular_z)/WHEEL_RADIUS, WHEEL_SEPARATION/(2.0*WHEEL_RADIUS), 
        cos(cmd_angular_z)/WHEEL_RADIUS, sin(cmd_angular_z)/WHEEL_RADIUS, -WHEEL_SEPARATION/(2*WHEEL_RADIUS);

  matrix_wheelAngularVelocityCommand = matrix_Jinv * matrix_wheelVelocityCommand;

  matrix_wheelVelocityResponse << wheel_velocity[0], wheel_velocity[1];

  //matrix_wheelVelocityResponse(0) = wheel_velocity[0];
  //matrix_wheelVelocityResponse(1) = wheel_velocity[1];

  matrix_wheelAngularAcc = matrix_wheelVelocityGain * ( matrix_wheelAngularVelocityCommand - matrix_wheelVelocityResponse);

  matrix_torque = matrix_inertia * matrix_wheelAngularAcc;

  int16_t dynamixel_current[2];
  int32_t dynamixel_current_of_2_wheels;
  myUnionForCurrent unionOfBytes;
  
  dynamixel_current[0] = (int16_t)(TORQUE_TO_CURRENT*matrix_torque(0));
  dynamixel_current[1] = (int16_t)(TORQUE_TO_CURRENT*matrix_torque(1));
  
  if ((dynamixel_current[0] <= CURRENT_LIMIT) && (dynamixel_current[1] <= CURRENT_LIMIT))
  {
    
    unionOfBytes.my2byteNumber[0] = dynamixel_current[0];
    unionOfBytes.my2byteNumber[1] = dynamixel_current[1];
    
    dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, wheel_id_array, NB_WHEEL, &unionOfBytes.my4byteNumber, 1, &log);
    //dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, &unionOfBytes.my4byteNumber, &log);
  }

  // Publish joint states
  if (millis() > publisher_timer) 
  {

    joint_states_msg.header.stamp = nh.now();
    joint_states_msg.header.frame_id = "kamal_robot";
    joint_states_msg.velocity_length = 2;
    joint_states_msg.position_length = 2;
    joint_states_msg.effort_length = 2;
    joint_states_msg.name_length = 2;
    joint_states_msg.name = wheel_names;
    joint_states_msg.position = wheel_position;
    joint_states_msg.velocity = wheel_velocity;
    joint_states_msg.effort = wheel_current;
  
    joint_states_pub.publish(&joint_states_msg);
    publisher_timer = millis() + 100; //publish ten times a second
    nh.spinOnce();
  }
  
  // Publish matrix data to debug 

 /* std_msgs::MultiArrayDimension myDim;
  //float myArray[4] = { matrix_wheelAngularAcc(0,0), matrix_wheelAngularAcc(1,0), matrix_torque(0,0), matrix_torque(1,0) };
  
  float myArray[2] = { dynamixel_current[0], dynamixel_current[1] };
  debug_matrix_msg.data_length = 2;
  debug_matrix_msg.data = myArray;
  debug_matrix_msg.layout.data_offset = 0;
  debug_matrix_msg.layout.dim = &myDim;
  debug_matrix_msg.layout.dim->label = "matrix_values";
  debug_matrix_msg.layout.dim->size = 2;
  debug_matrix_msg.layout.dim->stride = 2;
  debug_matrix_msg.layout.dim_length = 1;
 
  debug_matrix_pub.publish(&debug_matrix_msg);*/
  
  //nh.spinOnce();
  delay(1);
}
