#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>


/*
===========================================================
 ESP32 主从机械臂控制系统 FINAL

 功能：

 1. 四自由度主从机械臂
 2. 270°电位器匹配180°MG90S
 3. ADC滤波
 4. ADC死区
 5. 杜邦线掉线保持当前位置
 6. 舵机速度限制
 7. 平滑运动
 8. 按钮启动
 9. 蓝灯状态提示
10. 复位
11. Flash保存实际位置

===========================================================
*/



// ========================================================
// GPIO
// ========================================================


// 电位器

#define POT_BASE       32
#define POT_ARM        33
#define POT_FOREARM    34
#define POT_GRIPPER    35



// 舵机

#define SERVO_BASE     18
#define SERVO_ARM      19
#define SERVO_FOREARM  21
#define SERVO_GRIPPER  22



// 按钮

#define BUTTON_PIN     23



// LED

#define LED_PIN        2





// ========================================================
// 舵机对象
// ========================================================


Servo servoBase;
Servo servoArm;
Servo servoForearm;
Servo servoGripper;





// ========================================================
// Flash
// ========================================================


Preferences preferences;





// ========================================================
// 状态机
// ========================================================


enum SystemState
{

    IDLE,

    STARTING,

    RUNNING,

    RESETTING

};


SystemState systemState =
IDLE;





// ========================================================
// 复位角度
// ========================================================


const float RESET_BASE =
144;


const float RESET_ARM =
83;


const float RESET_FOREARM =
58;


const float RESET_GRIPPER =
22;





// ========================================================
// 舵机最大速度
// 度/秒
// ========================================================


const float BASE_MAX_SPEED =
50;


const float ARM_MAX_SPEED =
35;


const float FOREARM_MAX_SPEED =
40;


const float GRIPPER_MAX_SPEED =
80;





// ========================================================
// 电位器校准
//
// 270°电位器
// 使用中间180°
//
// 后期根据实际机械调节
// ========================================================



// -------- Base --------


int BASE_POT_MIN =
680;


int BASE_POT_MAX =
3415;


int BASE_SERVO_MIN =
0;


int BASE_SERVO_MAX =
180;


bool BASE_REVERSED =
false;


int BASE_OFFSET =
0;





// -------- Arm --------


int ARM_POT_MIN =
680;


int ARM_POT_MAX =
3415;


int ARM_SERVO_MIN =
0;


int ARM_SERVO_MAX =
180;


bool ARM_REVERSED =
false;


int ARM_OFFSET =
0;





// -------- Forearm --------


int FOREARM_POT_MIN =
680;


int FOREARM_POT_MAX =
3415;


int FOREARM_SERVO_MIN =
0;


int FOREARM_SERVO_MAX =
180;


bool FOREARM_REVERSED =
false;


int FOREARM_OFFSET =
0;





// -------- Gripper --------


int GRIPPER_POT_MIN =
900;


int GRIPPER_POT_MAX =
3200;


int GRIPPER_SERVO_MIN =
20;


int GRIPPER_SERVO_MAX =
120;


bool GRIPPER_REVERSED =
false;


int GRIPPER_OFFSET =
0;





// ========================================================
// 当前实际角度
// ========================================================


float currentBase =
RESET_BASE;


float currentArm =
RESET_ARM;


float currentForearm =
RESET_FOREARM;


float currentGripper =
RESET_GRIPPER;





// ========================================================
// 目标角度
// ========================================================


float targetBase =
RESET_BASE;


float targetArm =
RESET_ARM;


float targetForearm =
RESET_FOREARM;


float targetGripper =
RESET_GRIPPER;





// ========================================================
// ADC滤波
// ========================================================


const float FILTER_ALPHA =
0.15;



float filteredBase =
0;


float filteredArm =
0;


float filteredForearm =
0;


float filteredGripper =
0;





// ========================================================
// ADC死区
// ========================================================


const float ADC_DEAD_ZONE =
12;



float lastBaseADC =
0;


float lastArmADC =
0;


float lastForearmADC =
0;


float lastGripperADC =
0;





// ========================================================
// 掉线保护
// ========================================================


int baseErrorCount =
0;


int armErrorCount =
0;


int forearmErrorCount =
0;


int gripperErrorCount =
0;



const int MAX_ERROR_COUNT =
20;





// ========================================================
// 按钮
// ========================================================


bool lastButtonState =
HIGH;


unsigned long lastButtonTime =
0;


const unsigned long BUTTON_DEBOUNCE =
50;





// ========================================================
// LED
// ========================================================


unsigned long ledTimer =
0;


bool ledState =
false;


const unsigned long LED_BLINK_INTERVAL =
100;





// ========================================================
// 状态计时
// ========================================================


unsigned long stateStartTime =
0;


const unsigned long SYSTEM_SEQUENCE_TIME =
3000;





// ========================================================
// Flash保存
// ========================================================


unsigned long lastTargetChangeTime =
0;


float lastSavedBase =
0;


float lastSavedArm =
0;


float lastSavedForearm =
0;


float lastSavedGripper =
0;



const float SAVE_THRESHOLD =
2;



const unsigned long SAVE_DELAY =
1000;

// ========================================================
// ADC滤波读取
// ========================================================

float readFilteredADC(
    int pin,
    float &filteredValue
)
{

    int raw =
    analogRead(pin);


    if(filteredValue==0)
    {
        filteredValue=raw;
    }


    filteredValue =
    FILTER_ALPHA * raw +
    (1.0-FILTER_ALPHA)*filteredValue;


    return filteredValue;

}





// ========================================================
// ADC有效判断
// ========================================================

bool isADCValid(float value)
{

    if(value < 80)
        return false;


    if(value > 4015)
        return false;


    return true;

}





// ========================================================
// ADC转角度
// ========================================================

float convertToAngle(
    float adc,
    int potMin,
    int potMax,
    int servoMin,
    int servoMax,
    bool reversed,
    int offset
)
{

    adc =
    constrain(
        adc,
        potMin,
        potMax
    );


    float angle;



    if(!reversed)
    {

        angle =
        map(
            (long)adc,
            potMin,
            potMax,
            servoMin,
            servoMax
        );

    }

    else
    {

        angle =
        map(
            (long)adc,
            potMin,
            potMax,
            servoMax,
            servoMin
        );

    }


    angle += offset;


    return constrain(
        angle,
        0,
        180
    );

}





// ========================================================
// 平滑运动
// ========================================================

float moveSmooth(
    float current,
    float target,
    float speed,
    float dt
)
{

    float diff =
    target-current;



    if(abs(diff)<0.5)
    {
        return target;
    }



    float step =
    speed*dt;



    if(diff>step)
    {
        current+=step;
    }

    else if(diff<-step)
    {
        current-=step;
    }

    else
    {
        current=target;
    }


    return current;

}





// ========================================================
// 更新舵机
// ========================================================

void updateServos(
    float dt
)
{

    currentBase =
    moveSmooth(
        currentBase,
        targetBase,
        BASE_MAX_SPEED,
        dt
    );



    currentArm =
    moveSmooth(
        currentArm,
        targetArm,
        ARM_MAX_SPEED,
        dt
    );



    currentForearm =
    moveSmooth(
        currentForearm,
        targetForearm,
        FOREARM_MAX_SPEED,
        dt
    );



    currentGripper =
    moveSmooth(
        currentGripper,
        targetGripper,
        GRIPPER_MAX_SPEED,
        dt
    );





    servoBase.write(
        round(currentBase)
    );


    servoArm.write(
        round(currentArm)
    );


    servoForearm.write(
        round(currentForearm)
    );


    servoGripper.write(
        round(currentGripper)
    );

}





// ========================================================
// 读取主机械臂
// ========================================================

void readMasterArm()
{


// =====================================================
// Base
// =====================================================


float baseADC =
readFilteredADC(
POT_BASE,
filteredBase
);



if(isADCValid(baseADC))
{

    baseErrorCount=0;



    if(abs(baseADC-lastBaseADC)>ADC_DEAD_ZONE)
    {

        lastBaseADC=baseADC;



        targetBase =
        convertToAngle(
            baseADC,
            BASE_POT_MIN,
            BASE_POT_MAX,
            BASE_SERVO_MIN,
            BASE_SERVO_MAX,
            BASE_REVERSED,
            BASE_OFFSET
        );

    }

}

else
{

    baseErrorCount++;


    //掉线保持原目标

    if(baseErrorCount>=MAX_ERROR_COUNT)
    {

        targetBase =
        currentBase;

    }

}







// =====================================================
// Arm
// =====================================================


float armADC =
readFilteredADC(
POT_ARM,
filteredArm
);



if(isADCValid(armADC))
{

    armErrorCount=0;


    if(abs(armADC-lastArmADC)>ADC_DEAD_ZONE)
    {

        lastArmADC=armADC;



        targetArm =
        convertToAngle(
            armADC,
            ARM_POT_MIN,
            ARM_POT_MAX,
            ARM_SERVO_MIN,
            ARM_SERVO_MAX,
            ARM_REVERSED,
            ARM_OFFSET
        );

    }

}

else
{

    armErrorCount++;


    if(armErrorCount>=MAX_ERROR_COUNT)
    {

        targetArm =
        currentArm;

    }

}







// =====================================================
// Forearm
// =====================================================


float forearmADC =
readFilteredADC(
POT_FOREARM,
filteredForearm
);



if(isADCValid(forearmADC))
{

    forearmErrorCount=0;


    if(abs(forearmADC-lastForearmADC)>ADC_DEAD_ZONE)
    {

        lastForearmADC=forearmADC;



        targetForearm =
        convertToAngle(
            forearmADC,
            FOREARM_POT_MIN,
            FOREARM_POT_MAX,
            FOREARM_SERVO_MIN,
            FOREARM_SERVO_MAX,
            FOREARM_REVERSED,
            FOREARM_OFFSET
        );

    }

}

else
{

    forearmErrorCount++;


    if(forearmErrorCount>=MAX_ERROR_COUNT)
    {

        targetForearm =
        currentForearm;

    }

}







// =====================================================
// Gripper
// =====================================================


float gripperADC =
readFilteredADC(
POT_GRIPPER,
filteredGripper
);



if(isADCValid(gripperADC))
{

    gripperErrorCount=0;


    if(abs(gripperADC-lastGripperADC)>ADC_DEAD_ZONE)
    {

        lastGripperADC=gripperADC;



        targetGripper =
        convertToAngle(
            gripperADC,
            GRIPPER_POT_MIN,
            GRIPPER_POT_MAX,
            GRIPPER_SERVO_MIN,
            GRIPPER_SERVO_MAX,
            GRIPPER_REVERSED,
            GRIPPER_OFFSET
        );

    }

}

else
{

    gripperErrorCount++;


    if(gripperErrorCount>=MAX_ERROR_COUNT)
    {

        targetGripper =
        currentGripper;

    }

}


}







// ========================================================
// 保存当前位置
// ========================================================

void savePositions()
{

    preferences.putFloat(
        "base",
        currentBase
    );


    preferences.putFloat(
        "arm",
        currentArm
    );


    preferences.putFloat(
        "forearm",
        currentForearm
    );


    preferences.putFloat(
        "gripper",
        currentGripper
    );



    lastSavedBase=currentBase;

    lastSavedArm=currentArm;

    lastSavedForearm=currentForearm;

    lastSavedGripper=currentGripper;

}





void checkSave()
{


bool changed =

abs(currentBase-lastSavedBase)>SAVE_THRESHOLD ||

abs(currentArm-lastSavedArm)>SAVE_THRESHOLD ||

abs(currentForearm-lastSavedForearm)>SAVE_THRESHOLD ||

abs(currentGripper-lastSavedGripper)>SAVE_THRESHOLD;





if(changed)
{

    if(lastTargetChangeTime==0)
    {
        lastTargetChangeTime=millis();
    }



    if(
    millis()-lastTargetChangeTime
    >=SAVE_DELAY
    )
    {

        savePositions();

        lastTargetChangeTime=0;

    }

}

else
{

    lastTargetChangeTime=0;

}


}

// ========================================================
// LED快速闪烁
// ========================================================

void updateFastBlink()
{

    if(
        millis()-ledTimer
        >=LED_BLINK_INTERVAL
    )
    {

        ledTimer =
        millis();


        ledState =
        !ledState;


        digitalWrite(
            LED_PIN,
            ledState
        );

    }

}





// ========================================================
// 按钮检测
// ========================================================

bool buttonPressed()
{

    bool state =
    digitalRead(BUTTON_PIN);



    bool pressed =
    false;



    if(
        state==LOW &&
        lastButtonState==HIGH
    )
    {

        if(
            millis()-lastButtonTime
            >BUTTON_DEBOUNCE
        )
        {

            pressed=true;


            lastButtonTime=
            millis();

        }

    }



    lastButtonState=
    state;



    return pressed;

}





// ========================================================
// 启动系统
// ========================================================

void startSystem()
{

    systemState =
    STARTING;


    stateStartTime =
    millis();


    ledTimer =
    millis();


    ledState =
    false;



    Serial.println();
    Serial.println("================");
    Serial.println(" SYSTEM START ");
    Serial.println("================");

}





// ========================================================
// 复位系统
// ========================================================

void resetSystem()
{

    systemState =
    RESETTING;


    stateStartTime =
    millis();


    ledTimer =
    millis();


    ledState =
    false;




    targetBase =
    RESET_BASE;


    targetArm =
    RESET_ARM;


    targetForearm =
    RESET_FOREARM;


    targetGripper =
    RESET_GRIPPER;



    Serial.println();
    Serial.println("================");
    Serial.println(" SYSTEM RESET ");
    Serial.println("================");

}





// ========================================================
// setup
// ========================================================

void setup()
{

    Serial.begin(115200);



    analogReadResolution(12);





    //按钮

    pinMode(
        BUTTON_PIN,
        INPUT_PULLUP
    );





    //LED

    pinMode(
        LED_PIN,
        OUTPUT
    );


    digitalWrite(
        LED_PIN,
        LOW
    );







    //舵机

    servoBase.attach(
        SERVO_BASE
    );


    servoArm.attach(
        SERVO_ARM
    );


    servoForearm.attach(
        SERVO_FOREARM
    );


    servoGripper.attach(
        SERVO_GRIPPER
    );







    //Flash

    preferences.begin(
        "robotarm",
        false
    );





    targetBase =
    preferences.getFloat(
        "base",
        RESET_BASE
    );


    targetArm =
    preferences.getFloat(
        "arm",
        RESET_ARM
    );


    targetForearm =
    preferences.getFloat(
        "forearm",
        RESET_FOREARM
    );


    targetGripper =
    preferences.getFloat(
        "gripper",
        RESET_GRIPPER
    );







    currentBase =
    targetBase;


    currentArm =
    targetArm;


    currentForearm =
    targetForearm;


    currentGripper =
    targetGripper;







    servoBase.write(
        round(currentBase)
    );


    servoArm.write(
        round(currentArm)
    );


    servoForearm.write(
        round(currentForearm)
    );


    servoGripper.write(
        round(currentGripper)
    );







    lastSavedBase =
    currentBase;


    lastSavedArm =
    currentArm;


    lastSavedForearm =
    currentForearm;


    lastSavedGripper =
    currentGripper;





    Serial.println();
    Serial.println("==============================");
    Serial.println(" ESP32 MASTER SLAVE ARM ");
    Serial.println("==============================");

    Serial.println("STATE : IDLE");

    Serial.println("PRESS BUTTON");

}





// ========================================================
// loop
// ========================================================

void loop()
{

    static unsigned long lastTime =
    millis();



    unsigned long now =
    millis();



    float dt =
    (now-lastTime)/1000.0;



    lastTime =
    now;



    if(dt>0.1)
    {
        dt=0.1;
    }








// ========================================================
// IDLE
// ========================================================

if(systemState==IDLE)
{

    digitalWrite(
        LED_PIN,
        LOW
    );



    if(buttonPressed())
    {
        startSystem();
    }


    return;

}









// ========================================================
// STARTING
// ========================================================

if(systemState==STARTING)
{

    updateFastBlink();



    if(
    millis()-stateStartTime
    >=SYSTEM_SEQUENCE_TIME
    )
    {

        digitalWrite(
            LED_PIN,
            HIGH
        );


        systemState =
        RUNNING;



        Serial.println(
        "SYSTEM READY"
        );

    }


    return;

}









// ========================================================
// RUNNING
// ========================================================

if(systemState==RUNNING)
{

    if(buttonPressed())
    {

        resetSystem();


        return;

    }




    readMasterArm();



    updateServos(dt);



    checkSave();



    return;

}









// ========================================================
// RESETTING
// ========================================================

if(systemState==RESETTING)
{

    updateFastBlink();



    updateServos(dt);





    if(
    millis()-stateStartTime
    >=SYSTEM_SEQUENCE_TIME
    )
    {


        currentBase =
        RESET_BASE;


        currentArm =
        RESET_ARM;


        currentForearm =
        RESET_FOREARM;


        currentGripper =
        RESET_GRIPPER;






        targetBase =
        RESET_BASE;


        targetArm =
        RESET_ARM;


        targetForearm =
        RESET_FOREARM;


        targetGripper =
        RESET_GRIPPER;






        servoBase.write(
            RESET_BASE
        );


        servoArm.write(
            RESET_ARM
        );


        servoForearm.write(
            RESET_FOREARM
        );


        servoGripper.write(
            RESET_GRIPPER
        );





        savePositions();





        digitalWrite(
            LED_PIN,
            LOW
        );




        systemState =
        IDLE;



        Serial.println();

        Serial.println(
        "RESET COMPLETE"
        );

        Serial.println(
        "STATE IDLE"
        );

    }


    return;

}


}