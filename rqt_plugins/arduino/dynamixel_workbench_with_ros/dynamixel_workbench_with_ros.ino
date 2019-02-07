#include <DynamixelWorkbench.h>

#include <ros.h>
//#include <std_msgs/Int8.h>
//#include <std_msgs/Int16.h>
#include <rqt_plugins/Scan.h>
#include <rqt_plugins/PresentPosition.h>
#include <rqt_plugins/GoalTorque.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif          

#define BAUDRATE  115200
#define DXL_ID    1

DynamixelWorkbench dxl_wb;
ros::NodeHandle  nh;
rqt_plugins::PresentPosition present_position_msg;



bool result = false;
uint16_t model_number = 0;
uint8_t dxl_id = DXL_ID;
int32_t dxl_position = 0;

void messageGoalTorqueCb( const rqt_plugins::GoalTorque& torque_msg) {

    const char *log = NULL;
    result = dxl_wb.itemWrite(torque_msg.id,"Goal_Torque",(int32_t)torque_msg.goalTorque, &log);
}

// Create a service server that takes nothing and returns nothing
void callbackServiceScan(const rqt_plugins::Scan::Request & req, rqt_plugins::Scan::Response & res)
{
    const char *log = NULL;
    result = dxl_wb.ping(req.id, &model_number, &log);
    if (result != false)
    {
      res.modelNumber = model_number;
      res.dxl_string_comm_result.data = "";
      res.dxl_string_error.data = "";
    }
    else
    {
      res.modelNumber = 0;
      res.dxl_string_comm_result.data = log;
      res.dxl_string_error.data = log;
    }
}

ros::ServiceServer<rqt_plugins::Scan::Request, rqt_plugins::Scan::Response> serverScan("scan",&callbackServiceScan);
ros::Publisher pub_present_position("PresentPosition", &present_position_msg);
ros::Subscriber<rqt_plugins::GoalTorque> subGoalTorque("GoalTorque", messageGoalTorqueCb );

void setup() {
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor
  
  nh.initNode();
  nh.advertiseService(serverScan);
  nh.advertise(pub_present_position);
  nh.subscribe(subGoalTorque);
  
  const char *log = NULL;
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init");
    return;
  }
  else
  {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE);  
  }

  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to ping");
    return;
  }
  else
  {
    Serial.println("Succeeded to ping");
    Serial.print("id : ");
    Serial.println(dxl_id);
    Serial.print("model_number : ");
    Serial.println(model_number);
    Serial.print("model name : ");
    Serial.println(dxl_wb.getModelName(dxl_id));
  }

  result = dxl_wb.setTorqueControlMode(dxl_id, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to set the Torque Control Mode");
    return;
  }
  else
  {
    Serial.println("Succeeded to set The Torque Control Mode");
  }
  

}

void loop() {

  const char *log = NULL;
  result = dxl_wb.getPresentPositionData(dxl_id, &dxl_position, &log);

  if (result != false)
  {
    present_position_msg.presentPosition = dxl_position;
    present_position_msg.id = dxl_id;
  
    //nh.loginfo("Publish Present Position !");
    pub_present_position.publish(&present_position_msg);
  }

  
  nh.spinOnce();

}
