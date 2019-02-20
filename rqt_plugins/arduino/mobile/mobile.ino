#include <DynamixelWorkbench.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif  

#define BAUDRATE  57600
#define DXL_ID_WHEEL_LEFT    2
#define DXL_ID_WHEEL_RIGHT   3

DynamixelWorkbench dxl_wb;
ros::NodeHandle  nh;
sensor_msgs::JointState joint_states_msg;
ros::Publisher joint_states_pub("joint_states", &joint_states_msg, 100);


std::map<std::string, uint8_t> map_id_wheel_dynamixels;
float wheel_position[2];
float wheel_velocity[2];
float wheel_effort[2];

char *wheel_names[2];

void commandVelocityCallback(const geometry_msgs::Twist &msg)
{
  double robot_lin_vel = msg.linear.x;
  double robot_ang_vel = msg.angular.z;
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
  }
  else
  {
    Serial.print("Succeeded to init workbench with a baudrate : ");
    Serial.println(BAUDRATE);  
  }

  return result;
}

void getDynamixelsWheelInfo()
{
  map_id_wheel_dynamixels["wheel_right"] = DXL_ID_WHEEL_RIGHT;
  map_id_wheel_dynamixels["wheel_left"] = DXL_ID_WHEEL_LEFT;
  
  int i=0;
  std::map<std::string, uint8_t>::iterator it = map_id_wheel_dynamixels.begin();
  
  while (it != map_id_wheel_dynamixels.end())
  {
    strcpy(wheel_names[i],(it->first).c_str());
    it++; 
    i++;
  }
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
      Serial.println("Succeeded to ping");
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
    result = dxl_wb.wheelMode((uint8_t)it->second, 0, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.print("Failed to set the Wheel Mode ");
      Serial.print((it->first).c_str());
      Serial.println(" !");
      Serial.print("ID : ");
      Serial.println((uint8_t)it->second);
      return false;
    }
    else
    {
      Serial.println("Succeeded  to set the Wheel Mode");
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

bool initBulkReadWriteWheelDynamixels(void)
{
  bool result = false;
  const char* log;
  
  result = dxl_wb.initBulkWrite(&log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to init Bulk Write !");
    return false;
  }
  
  result = dxl_wb.initBulkRead(&log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to init Bulk Read !");
    return false;
  }

  result = dxl_wb.addBulkReadParam(map_id_wheel_dynamixels["wheel_right"], "Present_Position", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add bulk read position param for wheel right !");
    return false;
  }

  result = dxl_wb.addBulkReadParam(map_id_wheel_dynamixels["wheel_left"], "Present_Position", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add bulk read position param for wheel left !");
    return false;
  }

  result = dxl_wb.addBulkReadParam(map_id_wheel_dynamixels["wheel_right"], "Present_Velocity", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add bulk read velocity param for wheel right !");
    return false;
  }

  result = dxl_wb.addBulkReadParam(map_id_wheel_dynamixels["wheel_left"], "Present_Velocity", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add bulk read velocity param for wheel left !");
    return false;
  }

  result = dxl_wb.addBulkReadParam(map_id_wheel_dynamixels["wheel_right"], "Present_Current", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add bulk read current param for wheel right !");
    return false;
  }

  result = dxl_wb.addBulkReadParam(map_id_wheel_dynamixels["wheel_left"], "Present_Current", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add bulk read current param for wheel left !");
    return false;
  }  
  
  return true;
}

bool writeBulkGoalVelocityWheelDynamixels(int32_t wheel_right_velovity, int32_t wheel_left_velocity)
{
  bool result = false;
  const char* log;
  
  result = dxl_wb.addBulkWriteParam(map_id_wheel_dynamixels["wheel_right"], "Goal_Velocity", wheel_right_velovity, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add bulk write velocity param of wheel right !");
    return false;
  }

  result = dxl_wb.addBulkWriteParam(map_id_wheel_dynamixels["wheel_left"], "Goal_Velocity", wheel_left_velocity, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add bulk write velocity param of wheel left !");
    return false;
  }

  result = dxl_wb.bulkWrite(&log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to bulk write !");
    return false;
  }

  return true;
}

void setup() {
  
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor

  nh.advertise(joint_states_pub);
  nh.subscribe(cmd_vel_sub);

  if (!initWorkbench(DEVICE_NAME,BAUDRATE)) return;
  
  getDynamixelsWheelInfo();
  if (!loadWheelDynamixels()) return;
  if (!initWheelDynamixels()) return;

  if (!initBulkReadWriteWheelDynamixels()) return;
  
}

void loop() {
  
  int32_t present_data[6];
  bool result = false;
  const char* log;

  result = dxl_wb.getBulkReadData(&present_data[0], &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to Read Bulk Data !");
  }

  wheel_position[0] = present_data[0]; // wheel right position
  wheel_position[1] = present_data[1]; // wheel left position

  wheel_velocity[0] = present_data[2]; // wheel right velocity
  wheel_velocity[1] = present_data[3]; // wheel left velocity

  wheel_effort[0] = present_data[4]; // wheel right effort
  wheel_effort[1] = present_data[5]; // wheel left effort
  
  joint_states_msg.header.stamp = ros::Time::now();
  joint_states_msg.velocity_length = 2;
  joint_states_msg.position_length = 2;
  joint_states_msg.effort_length = 2;
  joint_states_msg.name_length = 2;
  joint_states_msg.position = wheel_position;
  joint_states_msg.velocity = wheel_velocity;
  joint_states_msg.effort = wheel_effort;
  joint_states_msg.name = wheel_names;
  
  joint_states_pub.publish(&joint_states_msg);

}
