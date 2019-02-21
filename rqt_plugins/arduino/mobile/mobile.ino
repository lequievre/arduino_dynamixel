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
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

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

DynamixelWorkbench dxl_wb;

ros::NodeHandle  nh;
sensor_msgs::JointState joint_states_msg;
ros::Publisher joint_states_pub("joint_states", &joint_states_msg);

std_msgs::Int32 present_position_msg;
ros::Publisher present_position_pub("PresentPosition", &present_position_msg);

std::map<std::string, uint8_t> map_id_wheel_dynamixels;

uint8_t wheel_id_array[2] = {DXL_ID_WHEEL_RIGHT, DXL_ID_WHEEL_LEFT};

int32_t wheel_raw_position[2] = {0, 0};
float wheel_position[2] = { 0.0, 0.0 };
int32_t wheel_raw_current[2] = {0, 0};
float wheel_current[2] = { 0.0, 0.0 };
int32_t wheel_raw_velocity[2] = {0, 0};
float wheel_velocity[2] = { 0.0, 0.0 };
char *wheel_names[2] = {"wheel_right", "wheel_left"};

const uint8_t handler_index = 0;


bool initWheelSyncWrite(void)
{
  bool result = false;
  const char* log;
  
  result = dxl_wb.addSyncWriteHandler(map_id_wheel_dynamixels["wheel_right"], "Goal_Velocity", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler goal velocity wheel right");
    //return false;
  }
  else
  {
    Serial.println("Succeeded to add sync write handler goal velocity wheel right");
  }
  
  result = dxl_wb.addSyncWriteHandler(map_id_wheel_dynamixels["wheel_left"], "Goal_Velocity", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler goal velocity wheel left");
    //return false;
  }
  else
  {
    Serial.println("Succeeded to add sync write handler goal velocity wheel left");
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

    wheel_current[0] = dxl_wb.convertValue2Current((int16_t)wheel_raw_current[0]);
    wheel_current[1] = dxl_wb.convertValue2Current((int16_t)wheel_raw_current[1]);
         
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

    wheel_position[0] = dxl_wb.convertValue2Radian(wheel_id_array[0], wheel_raw_position[0]); 
    wheel_position[1] = dxl_wb.convertValue2Radian(wheel_id_array[1], wheel_raw_position[1]);                                            

}

void commandVelocityCallback(const geometry_msgs::Twist &msg)
{
  bool result = false;
  const char* log;
  
  double robot_lin_vel = msg.linear.x;
  double robot_ang_vel = msg.angular.z;

  int32_t local_dynamixel_velocity[2] = { (int32_t)msg.linear.x, (int32_t)msg.linear.y};
  //int32_t local_dynamixel_velocity[2] = { 100, 100};

  result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, wheel_id_array, NB_WHEEL, local_dynamixel_velocity, 1, &log);

  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to sync write handler goal velocity");
  }
  else
  {
    Serial.println("Succeeded to sync write handler goal velocity");
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

void setup() {
  
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor
  
  nh.initNode();
  nh.advertise(joint_states_pub);
  nh.advertise(present_position_pub);
  nh.subscribe(cmd_vel_sub);

  if (!initWorkbench(DEVICE_NAME,BAUDRATE)) return;
  getDynamixelsWheelInfo();
  if (!loadWheelDynamixels()) return;
  initWheelDynamixels();

  if (!initWheelSyncRead()) return;
  initWheelSyncWrite();
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
