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

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT 1

// Protocol 2.0
#define ADDR_PRESENT_CURRENT_2 126
#define ADDR_PRESENT_VELOCITY_2 128
#define ADDR_PRESENT_POSITION_2 132

#define LENGTH_PRESENT_CURRENT_2 2
#define LENGTH_PRESENT_VELOCITY_2 4
#define LENGTH_PRESENT_POSITION_2 4


#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif  

#define BAUDRATE  57600
#define DXL_ID_WHEEL_RIGHT   2
#define DXL_ID_WHEEL_LEFT    3
#define NB_WHEEL             2

#define CURRENT_UNIT          3.36f   //  3.36[mA]
#define RPM_MX_64_2           0.229   // 0.229 rpm
#define WHEEL_RADIUS          0.047  // 4.7 cm
#define WHEEL_SEPARATION      0.21   // 21 cm

DynamixelWorkbench dxl_wb;

ros::NodeHandle  nh;
sensor_msgs::JointState joint_states_msg;
ros::Publisher joint_states_pub("joint_states", &joint_states_msg);

std::map<std::string, uint8_t> map_id_wheel_dynamixels;

uint8_t wheel_id_array[2] = {DXL_ID_WHEEL_RIGHT, DXL_ID_WHEEL_LEFT};

int32_t wheel_raw_position[2] = {0, 0};
float wheel_position[2] = { 0.0, 0.0 };
int32_t wheel_raw_current[2] = {0, 0};
float wheel_current[2] = { 0.0, 0.0 };
int32_t wheel_raw_velocity[2] = {0, 0};
float wheel_velocity[2] = { 0.0, 0.0 };
char *wheel_names[2] = {"wheel_right", "wheel_left"};

bool initWheelSyncWrite(void)
{
  bool result = false;
  const char* log;
  
  result = dxl_wb.addSyncWriteHandler(map_id_wheel_dynamixels["wheel_right"], "Goal_Current", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler goal current wheel right");
    //return false;
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
    //return false;
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

    wheel_raw_current[0] = 0;
    wheel_raw_current[1] = 0;

    union ToShort
    {
        uint32_t intValue;
        uint16_t shortValues[2];
    } toShort;
    
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

    //wheel_current[0] = dxl_wb.convertValue2Current((int16_t)wheel_raw_current[0]);
    //wheel_current[1] = dxl_wb.convertValue2Current((int16_t)wheel_raw_current[1]);
    //wheel_current[0] = wheel_raw_current[0] * CURRENT_UNIT;
    //wheel_current[1] = wheel_raw_current[1] * CURRENT_UNIT;

    int16_t inter[2];
    //inter[0] = (int16_t)wheel_raw_current[0];
    //inter[1] = (int16_t)wheel_raw_current[1];

   // memcpy( &inter[0], &wheel_raw_current[0], 2 );
   // memcpy( &inter[1], &wheel_raw_current[1], 2 );
    
   // wheel_current[0] = ((float)inter[0]) * 1.0;
   // wheel_current[1] = ((float)inter[1]) * 1.0;

   toShort.intValue = wheel_raw_current[1];
   int16_t shortValue = toShort.shortValues[0];

   wheel_current[0] = (int16_t)wheel_raw_current[0] * 1.0;
   wheel_current[1] = shortValue * 1.0;
         
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

  /*
    RPM = revolution per minute
    
    To convert RPM to rad/s, multiply by 0.10472 (which is an approximation of pi/30)
    RPM * 0.10472 = rad/s
    
    To convert rad/s to RPM, multiply by 9.54929 (which is an approximation of 30/pi)
    rad/s * 9.54929 = RPM
    
    The Angular to Linear Velocity formular is : v = r × ω

    Where:
      v: Linear velocity, in m/s
      r: Radius, in meter
      ω: Angular velocity, in rad/s
    
    The RPM to Linear Velocity formular is : v = r × RPM × 0.10472
    
    Where:
      v: Linear velocity, in m/s
      r: Radius, in meter
      RPM: Angular velocity, in RPM (Rounds per Minute)

    ==> v = r * w = r * (RPM * 0.10472)
    ==> v = r * ((RPM * Goal_Velocity) * 0.10472)    
    ==> Goal_Velocity = v / (r * RPM * 0.10472) = v * VELOCITY_CONSTANT_VALUE
      
   */

  bool result = false;
  const char* log;
  
  double robot_lin_vel = msg.linear.x;
  double robot_ang_vel = msg.angular.z;

  /*double wheel_velocity[2];
  int32_t dynamixel_velocity[2];
  const uint8_t LEFT = 1;
  const uint8_t RIGHT = 0;

  double velocity_constant_value = 1 / (WHEEL_RADIUS * RPM_MX_64_2 * 0.10472);

  wheel_velocity[RIGHT] = robot_lin_vel + (robot_ang_vel * WHEEL_SEPARATION / 2);
  wheel_velocity[LEFT]  = robot_lin_vel - (robot_ang_vel * WHEEL_SEPARATION / 2);

  dynamixel_velocity[RIGHT] = wheel_velocity[RIGHT] * velocity_constant_value;
  dynamixel_velocity[LEFT]  = wheel_velocity[LEFT] * velocity_constant_value;*/
  
  int32_t current_limit = 0;
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

  int32_t dynamixel_current[2];
  //dynamixel_current[0] = robot_lin_vel * 10000;
  //dynamixel_current[1] = robot_lin_vel * 10000;

  dynamixel_current[0] = msg.linear.x;
  dynamixel_current[1] = msg.linear.y;
  
  if ((dynamixel_current[0] > current_limit) || (dynamixel_current[0] > current_limit))
  {
    Serial.print("Error -> send command over current limit !");
  }
  else
  {
    result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, wheel_id_array, NB_WHEEL, dynamixel_current, 1, &log);
  
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to sync write handler goal current");
    }
    else
    {
      Serial.println("Succeeded to sync write handler goal current");
    }
  }
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
  nh.subscribe(cmd_vel_sub);

  if (!initWorkbench(DEVICE_NAME,BAUDRATE)) return;
  getDynamixelsWheelInfo();
  if (!loadWheelDynamixels()) return;
  initWheelDynamixels();

  if (!initWheelSyncRead()) return;
  initWheelSyncWrite();

  printWheelsInfos();
}

void loop() {

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
  
  nh.spinOnce();
  delay(1);
}
