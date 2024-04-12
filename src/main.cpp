#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <Servo.h>
#include <TaskScheduler.h>
#include "ICM20600.h"
#include <Wire.h>
 
/*--------------------------- Constantes ----------------------------*/
 
#define BAUD            115200    //Serial baud rate
 
#define PIN_MOT1         5
#define PIN_MOT2         6
#define PIN_MOT3         7

#define LIM_IN           22
#define LIM_OUT          23

#define ESTOP            2


/*------------------------- Globale variables -------------------------*/
//PPM motor outputs 
Servo motor1;
Servo motor2;
Servo motor3;

//Gyro
ICM20600 icm20600(true);
 
// Xbox Controller input variables
float LeftJoystickX_    = 0;
float LeftJoystickY_    = 0;
float RightJoystickX_   = 0;
float RightJoystickY_   = 0;
float LeftTrigger_      = 0;
float RightTrigger_     = 0;
bool  LeftBumper_       = 0;
bool  RightBumper_      = 0;
bool  AButton_          = 0;
bool  BButton_          = 0;
bool  XButton_          = 0;
bool  YButton_          = 0;
bool  LeftThumb_        = 0;
bool  RightThumb_       = 0;
bool  LeftDpad_         = 0;
bool  RightDpad_        = 0;
bool  UpDpad_           = 0;
bool  DownDpad_         = 0;
bool  BackButton_       = 0;
bool  StartButton_      = 0;
float M1, M2, M3;
bool BackButtonAct_     = false;
bool LimitIn_, LimitOut_    ;
float GyroX_ = 0.0;
float GyroY_ = 0.0;
float GyroZ_ = 0.0;

//Emergency stop variable
bool estop = true;
 
/*------------------------- Function prototypes -------------------------*/

void sendMsg();
void readMsg();
void serialEvent();
void desactivateMotors();

void motorUpdateCall();
void gyroUpdateCall();
void sendMsgCall();
void readMsgCall();

void estopCall();

void manageSerialCom();
float manageMotors(float inputUp,float inputDown, float minRange, float maxRange, float midRange);
float manageMotors(float input, float minRange, float maxRange, float midRange);

//Task shenanigans
Task MotorUpdateTask(50, TASK_FOREVER, &motorUpdateCall);
Task GyroUpdateCall(10, TASK_FOREVER, &gyroUpdateCall);
Task SendMsgTask(100, TASK_FOREVER, &sendMsgCall);
Task ReadMsgTask(0, TASK_FOREVER, &readMsgCall);

Scheduler Runner;
 
/*------------------------- Main function -------------------------*/
 
void setup() {
    //Adding and enabling tasks
    Runner.addTask(ReadMsgTask);
    Runner.addTask(SendMsgTask);
    SendMsgTask.enable();
    Runner.addTask(MotorUpdateTask);
    MotorUpdateTask.enable();
    Runner.addTask(GyroUpdateCall);
    GyroUpdateCall.enable();

    //Initializing motors
    motor1.attach(PIN_MOT1); //Triggers
    motor2.attach(PIN_MOT2); //leftJoystickX_
    motor3.attach(PIN_MOT3); //leftJoystickY_

    pinMode(LIM_IN, INPUT_PULLUP);    //Limit switch fully retracted
    pinMode(LIM_OUT, INPUT_PULLUP);   //Limit switch fully extended

    pinMode(ESTOP, INPUT_PULLUP);   //Emergency Stop Pin
    attachInterrupt(ESTOP, estopCall, RISING);
 
    Serial.begin(BAUD) ; // serial communication initialisation
    
    Wire.begin();
    icm20600.initialize();
    
    while(!Serial) {

    }
}
 
void loop() {
        Runner.execute();
}

void motorUpdateCall(){
    //Check if limit switches allow motor to move
    bool allowed = false;
    bool limIn = digitalRead(LIM_IN);
    bool limOut = digitalRead(LIM_OUT);

    if(!limIn && !limOut){
        allowed = true;
    }
    else if(limIn && (RightTrigger_ > 0)){
        allowed = true;
    }
    else if(limOut && (LeftTrigger_ > 0)){
        allowed = true;
    }

    //Update motors if enabled and allowed
    if(BackButtonAct_ && allowed && (!estop))
    {
        M1 = manageMotors(RightTrigger_,LeftTrigger_,1400,1600,1500);
        motor1.writeMicroseconds(M1);
    }
    //Set them to 0 speed if not enabled
    else
    {
        motor1.writeMicroseconds(1500);
    }

    if(BackButtonAct_ && (!estop)){
        M2 = manageMotors(LeftJoystickX_,1450,1600,1500);
        motor2.writeMicroseconds(M2);
    
        M3 = manageMotors(RightJoystickY_,1400,1600,1500);
        motor3.writeMicroseconds(M3);
    }
    else{
        motor2.writeMicroseconds(1500);
        motor2.writeMicroseconds(1500);
    }


    if(!digitalRead(ESTOP)){
        estop = false;
    }
}

void gyroUpdateCall(){
    GyroX_ += float(icm20600.getGyroscopeX()) * 0.01;
    GyroY_ += float(icm20600.getGyroscopeY()) * 0.01;
    GyroZ_ += float(icm20600.getGyroscopeZ()) * 0.01;
}

void resetGyro(){
    GyroX_ = 0.0;
    GyroY_ = 0.0;
    GyroZ_ = 0.0;
}

void serialEvent(){ReadMsgTask.enable();}

void readMsgCall(){
    readMsg();
    ReadMsgTask.disable();
}

void sendMsgCall(){
    sendMsg();
}

void estopCall(){
    estop = true;
}

/*------------------- Function definitions ----------------------*/

//Motor command calculation (2 inputs)
float manageMotors(float inputUp, float inputDown, float minRange, float maxRange, float midRange){

    //Calculates an overall input then uses the other function
    float input = inputUp-inputDown;
    float range = manageMotors(input, minRange, maxRange, midRange);
 
    return range;
}
 
//Motor command calculation (2 inputs)
float manageMotors(float input, float minRange, float maxRange, float midRange){

    float range;
 
    if(input>=0)
    {
        range = ((maxRange-midRange) * input) + midRange ;
    }else if(input<0){
        range = midRange+((midRange-minRange) * input);
    }
 
    return range;
}

//Send the JSON Message throught the serial port
void sendMsg(){
    JsonDocument doc;

    // Message Elements
    doc["LeftJoystickX"]    = LeftJoystickX_    ;
    //doc["LeftJoystickY"]    = LeftJoystickY_  ;
    //doc["RightJoystickX"]  = RightJoystickX_ ;
    doc["RightJoystickY"]  = RightJoystickY_   ;
    doc["LeftTrigger"]      = LeftTrigger_      ;
    doc["RightTrigger"]     = RightTrigger_     ;  
    /*
    doc["LeftBumper"]       = LeftBumper_       ;
    doc["RightBumper"]      = RightBumper_      ;
   */
    doc["AButton"]          = AButton_          ;
    
 /*
    doc["BButton"]          = BButton_          ;
    doc["XButton"]          = XButton_          ;
    doc["YButton"]          = YButton_          ;
    doc["LeftThumb"]        = LeftThumb_        ;
    doc["RightThumb"]       = RightThumb_       ;
    doc["LeftDpad"]         = LeftDpad_         ;
    doc["RightDpad"]        = RightDpad_        ;
    doc["UpDpad"]           = UpDpad_           ;
    doc["DownDpad"]         = DownDpad_         ;
    */
    doc["BackButton"]       = BackButton_       ;
 
    //doc["StartButton"]      = StartButton_      ;
 
    doc["Moteur1"]        = M1                          ;
    doc["Moteur2"]        = M2                          ;
    doc["Moteur3"]        = M3                          ;
    doc["BackButtonAct"]  = BackButtonAct_              ;
    doc["LimitIn"]        = LimitIn_                    ;
    doc["LimitOut"]       = LimitOut_                   ;
    doc["GyroX"]          = GyroX_                      ;
    doc["GyroY"]          = GyroY_                      ;
    doc["GyroZ"]          = GyroZ_                      ;


    //Serialization
    serializeJson(doc, Serial);
 
    //Sending JSON over serial
    Serial.println();
}

//Message decoding function
//Gets called on serial events
void readMsg(){
    // Reading Json message
    JsonDocument doc;
    JsonVariant parse_msg;
 
    // Reading Serial port
    DeserializationError error = deserializeJson(doc, Serial);
 
    // If Error in message
    if(error) {
        Serial.print("deserialize() failed:");
        Serial.println(error.c_str());
        return;
    }
 
    // Message analysis
 
    parse_msg = doc["LeftJoystickX"];
    if(!parse_msg.isNull()){
        LeftJoystickX_ = doc["LeftJoystickX"].as<float>();
    }
    /*
    parse_msg = doc["LeftJoystickY"];
    if(!parse_msg.isNull()){
        LeftJoystickY_ = doc["LeftJoystickY"].as<float>();
    }
 
    parse_msg = doc["RightJoystickX"];
    if(!parse_msg.isNull()){
        RightJoystickX_ = doc["RightJoystickX"].as<float>();
    }
    */
    parse_msg = doc["RightJoystickY"];
    if(!parse_msg.isNull()){
        RightJoystickY_ = doc["RightJoystickY"].as<float>();
    }
 
    parse_msg = doc["LeftTrigger"];
    if(!parse_msg.isNull()){
        LeftTrigger_ = doc["LeftTrigger"].as<float>();
    }
   
    parse_msg = doc["RightTrigger"];
    if(!parse_msg.isNull()){
        RightTrigger_ = doc["RightTrigger"].as<float>();
    }

    /*
    parse_msg = doc["LeftBumper"];
    if(!parse_msg.isNull()){
        LeftBumper_ = doc["LeftBumper"].as<bool>();
    }
 
    parse_msg = doc["RightBumper"];
    if(!parse_msg.isNull()){
        RightBumper_ = doc["RightBumper"].as<bool>();
    }
 
   */
    parse_msg = doc["AButton"];
    if(!parse_msg.isNull()){
        AButton_ = doc["AButton"].as<bool>();
    }
 /*
    parse_msg = doc["BButton"];
    if(!parse_msg.isNull()){
        BButton_ = doc["BButton"].as<bool>();
    }
 
    parse_msg = doc["XButton"];
    if(!parse_msg.isNull()){
        XButton_ = doc["XButton"].as<bool>();
    }
 
    parse_msg = doc["YButton"];
    if(!parse_msg.isNull()){
        YButton_ = doc["YButton"].as<bool>();
    }
   
    parse_msg = doc["LeftThumb"];
    if(!parse_msg.isNull()){
        LeftThumb_ = doc["LeftThumb"].as<bool>();
    }
 
    parse_msg = doc["RightThumb"];
    if(!parse_msg.isNull()){
        RightThumb_ = doc["RightThumb"].as<bool>();
    }
 
    parse_msg = doc["LeftDpad"];
    if(!parse_msg.isNull()){
        LeftDpad_ = doc["LeftDpad"].as<bool>();
    }
 
    parse_msg = doc["RightDpad"];
    if(!parse_msg.isNull()){
        RightDpad_ = doc["RightDpad"].as<bool>();
    }
 
    parse_msg = doc["UpDpad"];
    if(!parse_msg.isNull()){
        UpDpad_ = doc["UpDpad"].as<bool>();
    }
 
    parse_msg = doc["DownDpad"];
    if(!parse_msg.isNull()){
        DownDpad_ = doc["DownDpad"].as<bool>();
    }
    */
    
    parse_msg = doc["BackButton"];
    if(!parse_msg.isNull()){
        BackButton_ = doc["BackButton"].as<bool>();
    }
   /*
       parse_msg = doc["StartButton"];
    if(!parse_msg.isNull()){
        StartButton_ = doc["StartButton"].as<bool>();
    }
   */

   
    parse_msg = doc["BackButtonAct"];
    if(!parse_msg.isNull()){
        BackButtonAct_ = doc["BackButtonAct"].as<bool>();
    }
 
   
}
 
//===============
