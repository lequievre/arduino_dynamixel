#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif    

#define BAUDRATE  115200
#define DXL_ID    1

DynamixelWorkbench dxl_wb;

bool result = false;
uint16_t model_number = 0;
uint8_t dxl_id = DXL_ID;

void setup() {
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor
  
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

  result = dxl_wb.wheelMode(dxl_id, 0, &log);
  if (result == false)
  {
    Serial.println(log);
    return;
  }
  else
  {
    Serial.println(log);
  }
  int32_t goal = 15;
  result = dxl_wb.goalVelocity(dxl_id, (int32_t)goal, &log);
  if (result == false)
  {
    Serial.println(log);
    return;
  }
  else
  {
    Serial.println(log);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
