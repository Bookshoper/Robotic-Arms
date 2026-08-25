#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>

/*
===========================================================
        ESP32 主从机械臂控制系统
===========================================================

功能：

1. 4自由度主从机械臂
2. 电位器控制：
   Base      → GPIO32
   Arm       → GPIO33
   Forearm   → GPIO34
   Gripper   → GPIO35

3. 舵机：
   Base      → GPIO18
   Arm       → GPIO19
   Forearm   → GPIO21
   Gripper   → GPIO22

4. 启动按钮：
   GPIO23

5. ESP32板载蓝色LED：
   GPIO2

6. 通电后需要按按钮启动
7. 启动时蓝灯快速闪烁3秒
8. 3秒后蓝灯常亮，进入工作状态
9. 工作状态再次按按钮：
   机械臂回到指定位置
   Base    = 144°
   Arm     = 83°
   Forearm = 58°
   Gripper = 22°

10. 复位过程中蓝灯快速闪烁3秒
11. 复位完成后蓝灯熄灭，进入待机

12. 电位器滤波
13. 舵机平滑运动
14. 每个关节独立最大速度
15. 电位器异常/断线时保持当前舵机位置
16. 保存最后目标位置到ESP32 Flash

===========================================================
*/


// ========================================================
// ① GPIO定义
// ========================================================

// ---------- 电位器 ----------
const int POT_BASE    = 32;
const int POT_ARM     = 33;
const int POT_FOREARM = 34;
const int POT_GRIPPER = 35;


// ---------- 舵机 ----------
const int SERVO_BASE    = 18;
const int SERVO_ARM     = 19;
const int SERVO_FOREARM = 21;
const int SERVO_GRIPPER = 22;


// ---------- 按钮 ----------
const int BUTTON_PIN = 23;


// ---------- 板载蓝色LED ----------
const int LED_PIN = 2;


// ========================================================
// ② 创建舵机对象
// ========================================================

Servo servoBase;
Servo servoArm;
Servo servoForearm;
Servo servoGripper;


// ========================================================
// ③ Flash存储
// ========================================================

Preferences preferences;


// ========================================================
// ④ 系统状态
// ========================================================

enum SystemState
{
    IDLE,       // 待机
    STARTING,   // 启动自检
    RUNNING,    // 正常运行
    RESETTING   // 机械臂复位
};

SystemState systemState = IDLE;


// ========================================================
// ⑤ 复位位置
// ========================================================

const float RESET_BASE    = 144.0;
const float RESET_ARM     = 83.0;
const float RESET_FOREARM = 58.0;
const float RESET_GRIPPER = 22.0;


// ========================================================
// ⑥ 舵机最大速度
//
// 单位：度/秒
// ========================================================

const float BASE_MAX_SPEED    = 50.0;
const float ARM_MAX_SPEED     = 35.0;
const float FOREARM_MAX_SPEED = 40.0;
const float GRIPPER_MAX_SPEED = 80.0;


// ========================================================
// ⑦ 电位器校准参数
//
// !!! 这里以后需要根据你的实际机械结构调整 !!!
//
// potMin / potMax:
// 电位器实际ADC范围
//
// servoMin / servoMax:
// 对应舵机实际安全范围
//
// reversed:
// 是否反向
// ========================================================

// Base
int BASE_POT_MIN = 0;
int BASE_POT_MAX = 4095;

int BASE_SERVO_MIN = 10;
int BASE_SERVO_MAX = 170;

bool BASE_REVERSED = false;


// Arm
int ARM_POT_MIN = 0;
int ARM_POT_MAX = 4095;

int ARM_SERVO_MIN = 20;
int ARM_SERVO_MAX = 160;

bool ARM_REVERSED = false;


// Forearm
int FOREARM_POT_MIN = 0;
int FOREARM_POT_MAX = 4095;

int FOREARM_SERVO_MIN = 20;
int FOREARM_SERVO_MAX = 160;

bool FOREARM_REVERSED = false;


// Gripper
int GRIPPER_POT_MIN = 0;
int GRIPPER_POT_MAX = 4095;

int GRIPPER_SERVO_MIN = 22;
int GRIPPER_SERVO_MAX = 120;

bool GRIPPER_REVERSED = false;


// ========================================================
// ⑧ 当前角度
// ========================================================

float currentBase = RESET_BASE;
float currentArm = RESET_ARM;
float currentForearm = RESET_FOREARM;
float currentGripper = RESET_GRIPPER;


// ========================================================
// ⑨ 目标角度
// ========================================================

float targetBase = RESET_BASE;
float targetArm = RESET_ARM;
float targetForearm = RESET_FOREARM;
float targetGripper = RESET_GRIPPER;


// ========================================================
// ⑩ 电位器滤波
// ========================================================

// EMA滤波系数
// 越小越稳定，但反应越慢
// 越大越灵敏，但容易抖动

const float FILTER_ALPHA = 0.15;

float filteredBase = 0;
float filteredArm = 0;
float filteredForearm = 0;
float filteredGripper = 0;


// ========================================================
// ⑪ 按钮状态
// ========================================================

bool lastButtonState = HIGH;

unsigned long lastButtonTime = 0;

const unsigned long BUTTON_DEBOUNCE = 50;


// ========================================================
// ⑫ LED闪烁控制
// ========================================================

unsigned long ledTimer = 0;

bool ledState = false;

const unsigned long LED_BLINK_INTERVAL = 100;


// ========================================================
// ⑬ 系统启动/复位计时
// ========================================================

unsigned long stateStartTime = 0;

const unsigned long SYSTEM_SEQUENCE_TIME = 3000;


// ========================================================
// ⑭ Flash保存
// ========================================================

unsigned long lastTargetChangeTime = 0;

float lastSavedBase = 0;
float lastSavedArm = 0;
float lastSavedForearm = 0;
float lastSavedGripper = 0;

const float SAVE_THRESHOLD = 2.0;

const unsigned long SAVE_DELAY = 1000;


// ========================================================
// ⑮ 读取电位器
// ========================================================

float readFilteredADC(
    int pin,
    float &filteredValue
)
{
    int raw = analogRead(pin);

    // 第一次直接初始化
    if (filteredValue == 0)
    {
        filteredValue = raw;
    }

    // EMA滤波
    filteredValue =
        FILTER_ALPHA * raw +
        (1.0 - FILTER_ALPHA) * filteredValue;

    return filteredValue;
}


// ========================================================
// ⑯ 电位器是否有效
//
// 有效范围：
// ADC不能太接近0，也不能超过4095
//
// 如果你安装了10K下拉电阻，
// 断线时一般会接近0。
// ========================================================

bool isADCValid(float value)
{
    if (value < 50)
        return false;

    if (value > 4045)
        return false;

    return true;
}


// ========================================================
// ⑰ ADC → 舵机角度
// ========================================================

float convertToAngle(
    float adc,
    int potMin,
    int potMax,
    int servoMin,
    int servoMax,
    bool reversed
)
{
    adc = constrain(
        adc,
        potMin,
        potMax
    );

    float angle;

    if (!reversed)
    {
        angle = map(
            (long)adc,
            potMin,
            potMax,
            servoMin,
            servoMax
        );
    }
    else
    {
        angle = map(
            (long)adc,
            potMin,
            potMax,
            servoMax,
            servoMin
        );
    }

    return constrain(
        angle,
        servoMin,
        servoMax
    );
}


// ========================================================
// ⑱ 平滑移动
//
// 根据最大速度限制当前角度变化
// ========================================================

float moveSmooth(
    float current,
    float target,
    float maxSpeed,
    float deltaTime
)
{
    float difference = target - current;

    // 小于0.5°直接认为到达
    if (abs(difference) < 0.5)
    {
        return target;
    }

    float maxStep =
        maxSpeed * deltaTime;

    if (difference > maxStep)
    {
        current += maxStep;
    }
    else if (difference < -maxStep)
    {
        current -= maxStep;
    }
    else
    {
        current = target;
    }

    return current;
}


// ========================================================
// ⑲ 更新舵机
// ========================================================

void updateServos(float deltaTime)
{
    currentBase =
        moveSmooth(
            currentBase,
            targetBase,
            BASE_MAX_SPEED,
            deltaTime
        );

    currentArm =
        moveSmooth(
            currentArm,
            targetArm,
            ARM_MAX_SPEED,
            deltaTime
        );

    currentForearm =
        moveSmooth(
            currentForearm,
            targetForearm,
            FOREARM_MAX_SPEED,
            deltaTime
        );

    currentGripper =
        moveSmooth(
            currentGripper,
            targetGripper,
            GRIPPER_MAX_SPEED,
            deltaTime
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
// ⑳ 读取主机械臂
// ========================================================

void readMasterArm()
{
    // ---------- Base ----------
    float baseADC =
        readFilteredADC(
            POT_BASE,
            filteredBase
        );

    if (isADCValid(baseADC))
    {
        targetBase =
            convertToAngle(
                baseADC,
                BASE_POT_MIN,
                BASE_POT_MAX,
                BASE_SERVO_MIN,
                BASE_SERVO_MAX,
                BASE_REVERSED
            );
    }


    // ---------- Arm ----------
    float armADC =
        readFilteredADC(
            POT_ARM,
            filteredArm
        );

    if (isADCValid(armADC))
    {
        targetArm =
            convertToAngle(
                armADC,
                ARM_POT_MIN,
                ARM_POT_MAX,
                ARM_SERVO_MIN,
                ARM_SERVO_MAX,
                ARM_REVERSED
            );
    }


    // ---------- Forearm ----------
    float forearmADC =
        readFilteredADC(
            POT_FOREARM,
            filteredForearm
        );

    if (isADCValid(forearmADC))
    {
        targetForearm =
            convertToAngle(
                forearmADC,
                FOREARM_POT_MIN,
                FOREARM_POT_MAX,
                FOREARM_SERVO_MIN,
                FOREARM_SERVO_MAX,
                FOREARM_REVERSED
            );
    }


    // ---------- Gripper ----------
    float gripperADC =
        readFilteredADC(
            POT_GRIPPER,
            filteredGripper
        );

    if (isADCValid(gripperADC))
    {
        targetGripper =
            convertToAngle(
                gripperADC,
                GRIPPER_POT_MIN,
                GRIPPER_POT_MAX,
                GRIPPER_SERVO_MIN,
                GRIPPER_SERVO_MAX,
                GRIPPER_REVERSED
            );
    }
}


// ========================================================
// ㉑ 保存当前位置
// ========================================================

void savePositions()
{
    preferences.putFloat(
        "base",
        targetBase
    );

    preferences.putFloat(
        "arm",
        targetArm
    );

    preferences.putFloat(
        "forearm",
        targetForearm
    );

    preferences.putFloat(
        "gripper",
        targetGripper
    );

    lastSavedBase = targetBase;
    lastSavedArm = targetArm;
    lastSavedForearm = targetForearm;
    lastSavedGripper = targetGripper;
}


// ========================================================
// ㉒ 判断是否需要保存
// ========================================================

void checkSave()
{
    bool changed =
        abs(targetBase - lastSavedBase) > SAVE_THRESHOLD ||
        abs(targetArm - lastSavedArm) > SAVE_THRESHOLD ||
        abs(targetForearm - lastSavedForearm) > SAVE_THRESHOLD ||
        abs(targetGripper - lastSavedGripper) > SAVE_THRESHOLD;


    if (changed)
    {
        if (lastTargetChangeTime == 0)
        {
            lastTargetChangeTime = millis();
        }

        if (
            millis() - lastTargetChangeTime
            >= SAVE_DELAY
        )
        {
            savePositions();

            lastTargetChangeTime = 0;
        }
    }
    else
    {
        lastTargetChangeTime = 0;
    }
}


// ========================================================
// ㉓ LED快速闪烁
// ========================================================

void updateFastBlink()
{
    if (
        millis() - ledTimer
        >= LED_BLINK_INTERVAL
    )
    {
        ledTimer = millis();

        ledState = !ledState;

        digitalWrite(
            LED_PIN,
            ledState
        );
    }
}


// ========================================================
// ㉔ 按钮检测
// ========================================================

bool buttonPressed()
{
    bool currentState =
        digitalRead(BUTTON_PIN);

    bool pressed = false;

    if (
        currentState == LOW &&
        lastButtonState == HIGH
    )
    {
        if (
            millis() - lastButtonTime
            > BUTTON_DEBOUNCE
        )
        {
            pressed = true;

            lastButtonTime = millis();
        }
    }

    lastButtonState = currentState;

    return pressed;
}


// ========================================================
// ㉕ 开始启动
// ========================================================

void startSystem()
{
    systemState = STARTING;

    stateStartTime = millis();

    ledTimer = millis();

    ledState = false;

    Serial.println();
    Serial.println("==============================");
    Serial.println(" SYSTEM STARTING");
    Serial.println("==============================");
}


// ========================================================
// ㉖ 开始复位
// ========================================================

void resetSystem()
{
    systemState = RESETTING;

    stateStartTime = millis();

    ledTimer = millis();

    ledState = false;


    // 设置复位目标
    targetBase = RESET_BASE;
    targetArm = RESET_ARM;
    targetForearm = RESET_FOREARM;
    targetGripper = RESET_GRIPPER;


    Serial.println();
    Serial.println("==============================");
    Serial.println(" SYSTEM RESET");
    Serial.println("==============================");
}


// ========================================================
// ㉗ setup
// ========================================================

void setup()
{
    Serial.begin(115200);


    // ---------- ADC ----------
    analogReadResolution(12);


    // ---------- 按钮 ----------
    pinMode(
        BUTTON_PIN,
        INPUT_PULLUP
    );


    // ---------- LED ----------
    pinMode(
        LED_PIN,
        OUTPUT
    );

    digitalWrite(
        LED_PIN,
        LOW
    );


    // ---------- 舵机 ----------
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


    // ---------- Flash ----------
    preferences.begin(
        "arm",
        false
    );


    // 读取上一次保存的位置
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


    // 当前角度先设置为保存的位置
    currentBase = targetBase;
    currentArm = targetArm;
    currentForearm = targetForearm;
    currentGripper = targetGripper;


    // 舵机输出
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


    lastSavedBase = targetBase;
    lastSavedArm = targetArm;
    lastSavedForearm = targetForearm;
    lastSavedGripper = targetGripper;


    Serial.println();
    Serial.println("==============================");
    Serial.println(" Master Slave Robotic Arm");
    Serial.println("==============================");
    Serial.println("System: IDLE");
    Serial.println("Press button to START");
    Serial.println();
}


// ========================================================
// ㉘ loop
// ========================================================

void loop()
{
    static unsigned long lastLoopTime = millis();

    unsigned long now = millis();

    float deltaTime =
        (now - lastLoopTime) / 1000.0;

    lastLoopTime = now;


    // 防止极端情况下deltaTime过大
    if (deltaTime > 0.1)
    {
        deltaTime = 0.1;
    }


    // ====================================================
    // IDLE
    // ====================================================

    if (systemState == IDLE)
    {
        digitalWrite(
            LED_PIN,
            LOW
        );


        if (buttonPressed())
        {
            startSystem();
        }


        return;
    }


    // ====================================================
    // STARTING
    // ====================================================

    if (systemState == STARTING)
    {
        updateFastBlink();


        // 3秒自检
        if (
            millis() - stateStartTime
            >= SYSTEM_SEQUENCE_TIME
        )
        {
            digitalWrite(
                LED_PIN,
                HIGH
            );


            systemState = RUNNING;


            Serial.println();
            Serial.println("SYSTEM READY");
            Serial.println("SYSTEM RUNNING");
            Serial.println();
        }


        return;
    }


    // ====================================================
    // RUNNING
    // ====================================================

    if (systemState == RUNNING)
    {
        // 再次按按钮 → 复位
        if (buttonPressed())
        {
            resetSystem();

            return;
        }


        // 读取主机械臂
        readMasterArm();


        // 平滑控制从机械臂
        updateServos(
            deltaTime
        );


        // 保存目标位置
        checkSave();


        return;
    }


    // ====================================================
    // RESETTING
    // ====================================================

    if (systemState == RESETTING)
    {
        // LED快速闪烁
        updateFastBlink();


        // 机械臂向复位位置移动
        updateServos(
            deltaTime
        );


        // 3秒结束
        if (
            millis() - stateStartTime
            >= SYSTEM_SEQUENCE_TIME
        )
        {
            // 确保最终位置准确
            currentBase = RESET_BASE;
            currentArm = RESET_ARM;
            currentForearm = RESET_FOREARM;
            currentGripper = RESET_GRIPPER;


            targetBase = RESET_BASE;
            targetArm = RESET_ARM;
            targetForearm = RESET_FOREARM;
            targetGripper = RESET_GRIPPER;


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


            // 保存复位位置
            savePositions();


            // LED关闭
            digitalWrite(
                LED_PIN,
                LOW
            );


            // 回到待机
            systemState = IDLE;


            Serial.println();
            Serial.println("==============================");
            Serial.println(" RESET COMPLETE");
            Serial.println(" System: IDLE");
            Serial.println("==============================");
            Serial.println();
        }


        return;
    }
}