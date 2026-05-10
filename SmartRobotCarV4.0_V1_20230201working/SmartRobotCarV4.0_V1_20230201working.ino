/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-18 14:14:35
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath:
 * @Modified: Ported to Arduino Uno R4 WiFi
 */
#include "ApplicationFunctionSet_xxx0.h"
#include "WiFiControl_xxx0.h"

void setup() {
  // put your setup code here, to run once:
  analogReadResolution(10); // R4 defaults to 14-bit; set to 10-bit for
                            // compatibility with original voltage formulas
  Application_FunctionSet.ApplicationFunctionSet_Init();

  // Wait for USB serial with timeout (Pi may not trigger DTR handshake)
  unsigned long serialWait = millis();
  while (!Serial && (millis() - serialWait < 3000)) {
    ; // wait up to 3 seconds
  }

  WiFiControl_Init();
  Serial.println("SYSTEM_READY");
}

void loop() {
  // put your main code here, to run repeatedly :
  Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
  Application_FunctionSet.ApplicationFunctionSet_KeyCommand();
  Application_FunctionSet.ApplicationFunctionSet_RGB();
  Application_FunctionSet.ApplicationFunctionSet_Follow();
  Application_FunctionSet.ApplicationFunctionSet_Obstacle();
  Application_FunctionSet.ApplicationFunctionSet_Tracking();
  Application_FunctionSet.ApplicationFunctionSet_Rocker();
  Application_FunctionSet.ApplicationFunctionSet_Standby();
  Application_FunctionSet.ApplicationFunctionSet_IRrecv();
  Application_FunctionSet.ApplicationFunctionSet_SerialPortDataAnalysis();

  Application_FunctionSet.CMD_ServoControl_xxx0();
  Application_FunctionSet.CMD_MotorControl_xxx0();
  Application_FunctionSet.CMD_CarControlTimeLimit_xxx0();
  Application_FunctionSet.CMD_CarControlNoTimeLimit_xxx0();
  Application_FunctionSet.CMD_MotorControlSpeed_xxx0();
  Application_FunctionSet.CMD_LightingControlTimeLimit_xxx0();
  Application_FunctionSet.CMD_LightingControlNoTimeLimit_xxx0();
  Application_FunctionSet.CMD_ClearAllFunctions_xxx0();

  WiFiControl_Loop();
}
