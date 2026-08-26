#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

/*
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
*/

// ================== GPIO 定义 ==================
#define POT_BASE       32
#define POT_ARM        33
#define POT_FOREARM    34
#define POT_GRIPPER    35

#define SERVO_BASE     18
#define SERVO_ARM      19
#define SERVO_FOREARM  21
#define SERVO_GRIPPER  22

#define BUTTON_PIN     23
#define LED_PIN        2

// ================== 对象声明 ==================
Servo servoBase;
Servo servoArm;
Servo servoForearm;
Servo servoGripper;

Preferences preferences;
WebServer server(80);

const char* AP_SSID = "ESP32-RobotArm";
const char* AP_PASSWORD = "12345678";
String systemStatus = "IDLE";

// ================== 系统状态 ==================
enum SystemState { IDLE, STARTING, RUNNING, RESETTING };
SystemState systemState = IDLE;

// ================== 物理参数设置 ==================
const float RESET_BASE = 144;
const float RESET_ARM = 83;
const float RESET_FOREARM = 58;
const float RESET_GRIPPER = 60;

// 舵机最大速度 (度/秒)
const float BASE_MAX_SPEED = 50;
const float ARM_MAX_SPEED = 35;
const float FOREARM_MAX_SPEED = 40;
const float GRIPPER_MAX_SPEED = 80;

// 电位器及舵机校准参数 (最小值, 最大值, 舵机最小, 舵机最大, 是否反转, 偏移)
int CALIB_BASE[6]    = {680, 3415, 0,  180, false, 0};
int CALIB_ARM[6]     = {680, 3415, 0,  180, false, 0};
int CALIB_FOREARM[6] = {680, 3415, 0,  180, false, 0};
int CALIB_GRIPPER[6] = {900, 3200, 20, 120, false, 0};

// 当前及目标角度
float currentBase = RESET_BASE, targetBase = RESET_BASE;
float currentArm = RESET_ARM, targetArm = RESET_ARM;
float currentForearm = RESET_FOREARM, targetForearm = RESET_FOREARM;
float currentGripper = RESET_GRIPPER, targetGripper = RESET_GRIPPER;

// ADC滤波及死区
const float FILTER_ALPHA = 0.15;
const float ANGLE_DEAD_ZONE = 1.0; // 角度变化大于1度才执行更新，抗抖动核心！

float filteredBase = 0, filteredArm = 0, filteredForearm = 0, filteredGripper = 0;
int baseErrorCount = 0, armErrorCount = 0, forearmErrorCount = 0, gripperErrorCount = 0;
const int MAX_ERROR_COUNT = 20;

// 硬件状态变量
bool lastButtonState = HIGH;
unsigned long lastButtonTime = 0;
const unsigned long BUTTON_DEBOUNCE = 50;

unsigned long ledTimer = 0;
bool ledState = false;
const unsigned long LED_BLINK_INTERVAL = 100;
unsigned long stateStartTime = 0;
const unsigned long SYSTEM_SEQUENCE_TIME = 3000;

// Flash 保存变量
unsigned long lastTargetChangeTime = 0;
float lastSavedBase = 0, lastSavedArm = 0, lastSavedForearm = 0, lastSavedGripper = 0;
const float SAVE_THRESHOLD = 2.0;
const unsigned long SAVE_DELAY = 1000;

// 多线程任务句柄
TaskHandle_t WebServerTask;

// ================== 功能函数 ==================

float readFilteredADC(int pin, float &filteredValue) {
    int raw = analogRead(pin);
    if(filteredValue == 0) filteredValue = raw;
    filteredValue = FILTER_ALPHA * raw + (1.0 - FILTER_ALPHA) * filteredValue;
    return filteredValue;
}

bool isADCValid(float value) {
    return (value >= 80 && value <= 4015);
}

float convertToAngle(float adc, int* calib) {
    adc = constrain(adc, calib[0], calib[1]);
    float angle;
    if(!calib[4]) { // 不反转
        angle = map((long)adc, calib[0], calib[1], calib[2], calib[3]);
    } else { // 反转
        angle = map((long)adc, calib[0], calib[1], calib[3], calib[2]);
    }
    angle += calib[5];
    return constrain(angle, 0, 180);
}

float moveSmooth(float current, float target, float speed, float dt) {
    float diff = target - current;
    if(abs(diff) < 0.5) return target;
    
    float step = speed * dt;
    if(diff > step) current += step;
    else if(diff < -step) current -= step;
    else current = target;
    
    return current;
}

void updateServos(float dt) {
    currentBase = moveSmooth(currentBase, targetBase, BASE_MAX_SPEED, dt);
    currentArm = moveSmooth(currentArm, targetArm, ARM_MAX_SPEED, dt);
    currentForearm = moveSmooth(currentForearm, targetForearm, FOREARM_MAX_SPEED, dt);
    currentGripper = moveSmooth(currentGripper, targetGripper, GRIPPER_MAX_SPEED, dt);

    servoBase.write(round(currentBase));
    servoArm.write(round(currentArm));
    servoForearm.write(round(currentForearm));
    servoGripper.write(round(currentGripper));
}

void updateChannel(int pin, float &filteredADC, int &errorCount, float &targetAngle, float currentAngle, int* calib) {
    float rawADC = readFilteredADC(pin, filteredADC);
    
    if(isADCValid(rawADC)) {
        errorCount = 0;
        float newTarget = convertToAngle(rawADC, calib);
        
        // 角度滞回死区：只有变化超过阈值，才更新目标角度
        if(abs(newTarget - targetAngle) > ANGLE_DEAD_ZONE) {
            targetAngle = newTarget;
        }
    } else {
        errorCount++;
        if(errorCount >= MAX_ERROR_COUNT) {
            targetAngle = currentAngle; // 掉线保护，保持当前位置
        }
    }
}

void readMasterArm() {
    updateChannel(POT_BASE, filteredBase, baseErrorCount, targetBase, currentBase, CALIB_BASE);
    updateChannel(POT_ARM, filteredArm, armErrorCount, targetArm, currentArm, CALIB_ARM);
    updateChannel(POT_FOREARM, filteredForearm, forearmErrorCount, targetForearm, currentForearm, CALIB_FOREARM);
    updateChannel(POT_GRIPPER, filteredGripper, gripperErrorCount, targetGripper, currentGripper, CALIB_GRIPPER);
}

void savePositions() {
    preferences.putFloat("base", currentBase);
    preferences.putFloat("arm", currentArm);
    preferences.putFloat("forearm", currentForearm);
    preferences.putFloat("gripper", currentGripper);

    lastSavedBase = currentBase;
    lastSavedArm = currentArm;
    lastSavedForearm = currentForearm;
    lastSavedGripper = currentGripper;
}

void checkSave() {
    bool changed = abs(currentBase - lastSavedBase) > SAVE_THRESHOLD ||
                   abs(currentArm - lastSavedArm) > SAVE_THRESHOLD ||
                   abs(currentForearm - lastSavedForearm) > SAVE_THRESHOLD ||
                   abs(currentGripper - lastSavedGripper) > SAVE_THRESHOLD;

    if(changed) {
        if(lastTargetChangeTime == 0) lastTargetChangeTime = millis();
        if(millis() - lastTargetChangeTime >= SAVE_DELAY) {
            savePositions();
            lastTargetChangeTime = 0;
        }
    } else {
        lastTargetChangeTime = 0;
    }
}

void updateFastBlink() {
    if(millis() - ledTimer >= LED_BLINK_INTERVAL) {
        ledTimer = millis();
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }
}

bool buttonPressed() {
    bool state = digitalRead(BUTTON_PIN);
    bool pressed = false;
    if(state == LOW && lastButtonState == HIGH) {
        if(millis() - lastButtonTime > BUTTON_DEBOUNCE) {
            pressed = true;
            lastButtonTime = millis();
        }
    }
    lastButtonState = state;
    return pressed;
}

void startSystem() {
    systemState = STARTING;
    systemStatus = "STARTING";
    stateStartTime = millis();
    ledTimer = millis();
    ledState = false;
    Serial.println("\n================\n SYSTEM START \n================");
}

void resetSystem() {
    systemState = RESETTING;
    systemStatus = "RESETTING";
    stateStartTime = millis();
    ledTimer = millis();
    ledState = false;

    targetBase = RESET_BASE;
    targetArm = RESET_ARM;
    targetForearm = RESET_FOREARM;
    targetGripper = RESET_GRIPPER;
    Serial.println("\n================\n SYSTEM RESET \n================");
}

// ================== Web 服务相关 ==================

void handleRoot() {

    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>Robot Arm Monitor</title>

<style>

body{
    margin:0;
    background:#121212;
    color:white;
    font-family:"Arial";
    text-align:center;
}

h1{
    margin-top:25px;
}

.card{

    background:#1e1e1e;
    margin:15px auto;
    padding:20px;
    width:85%;
    max-width:500px;
    border-radius:20px;
    box-shadow:0 0 15px #000;

}


.title{

    font-size:22px;
    color:#00e5ff;

}


.value{

    font-size:30px;
    color:#00ff88;
    margin:8px;

}


.status{

    font-size:35px;
    color:#ffcc00;

}


.item{

    display:flex;
    justify-content:space-between;
    font-size:20px;
    margin:10px;

}


</style>


</head>


<body>


<h1>🤖 ESP32 Robot Arm</h1>



<div class="card">

<div class="title">
System Status
</div>

<div id="state" class="status">
Loading...
</div>

</div>



<div class="card">

<div class="title">
Servo Angle
</div>


<div class="item">
Base
<span id="base" class="value">0</span>
</div>


<div class="item">
Arm
<span id="arm" class="value">0</span>
</div>


<div class="item">
Forearm
<span id="forearm" class="value">0</span>
</div>


<div class="item">
Gripper
<span id="gripper" class="value">0</span>
</div>


</div>





<div class="card">

<div class="title">
Potentiometer ADC
</div>


<div class="item">
Base
<span id="adcbase" class="value">0</span>
</div>


<div class="item">
Arm
<span id="adcarm" class="value">0</span>
</div>


<div class="item">
Forearm
<span id="adcforearm" class="value">0</span>
</div>


<div class="item">
Gripper
<span id="adcgripper" class="value">0</span>
</div>



</div>



<script>


function updateData(){

fetch('/status')

.then(response=>response.json())

.then(data=>{


document.getElementById("state").innerHTML=data.state;


document.getElementById("base").innerHTML=
data.base.toFixed(1)+"°";


document.getElementById("arm").innerHTML=
data.arm.toFixed(1)+"°";


document.getElementById("forearm").innerHTML=
data.forearm.toFixed(1)+"°";


document.getElementById("gripper").innerHTML=
data.gripper.toFixed(1)+"°";



document.getElementById("adcbase").innerHTML=
data.adcbase.toFixed(0);


document.getElementById("adcarm").innerHTML=
data.adcarm.toFixed(0);


document.getElementById("adcforearm").innerHTML=
data.adcforearm.toFixed(0);


document.getElementById("adcgripper").innerHTML=
data.adcgripper.toFixed(0);



});


}



setInterval(updateData,500);


</script>



</body>
</html>

)rawliteral";


server.send(200,"text/html",html);

}

void handleStatus() {
    char json[256];
    snprintf(json, sizeof(json), 
        "{\"state\":\"%s\",\"base\":%.1f,\"arm\":%.1f,\"forearm\":%.1f,\"gripper\":%.1f,\"adcbase\":%.1f,\"adcarm\":%.1f,\"adcforearm\":%.1f,\"adcgripper\":%.1f}",
        systemStatus.c_str(), currentBase, currentArm, currentForearm, currentGripper, 
        filteredBase, filteredArm, filteredForearm, filteredGripper);
    server.send(200, "application/json", json);
}

// 独立的 WiFi 任务运行在 Core 0
void webServerTaskFunc(void *pvParameters) {
    for (;;) {
        server.handleClient();
        vTaskDelay(10 / portTICK_PERIOD_MS); // 释放CPU，防止看门狗复位
    }
}

// ================== 主程序 ==================

void setup() {
    Serial.begin(115200);

    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.println("\nWiFi AP Started. IP Address: " + WiFi.softAPIP().toString());

    server.on("/", handleRoot);
    server.on("/status", handleStatus);
    server.begin();
    
    // 在 Core 0 上启动 WebServer 任务
    xTaskCreatePinnedToCore(webServerTaskFunc, "WebServerTask", 4096, NULL, 1, &WebServerTask, 0);

    analogReadResolution(12);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    servoBase.attach(SERVO_BASE);
    servoArm.attach(SERVO_ARM);
    servoForearm.attach(SERVO_FOREARM);
    servoGripper.attach(SERVO_GRIPPER);

    preferences.begin("robotarm", false);
    currentBase = targetBase = preferences.getFloat("base", RESET_BASE);
    currentArm = targetArm = preferences.getFloat("arm", RESET_ARM);
    currentForearm = targetForearm = preferences.getFloat("forearm", RESET_FOREARM);
    currentGripper = targetGripper = preferences.getFloat("gripper", RESET_GRIPPER);

    servoBase.write(round(currentBase));
    servoArm.write(round(currentArm));
    servoForearm.write(round(currentForearm));
    servoGripper.write(round(currentGripper));

    lastSavedBase = currentBase;
    lastSavedArm = currentArm;
    lastSavedForearm = currentForearm;
    lastSavedGripper = currentGripper;

    Serial.println("\n==============================\n ESP32 MASTER SLAVE ARM \n==============================");
    Serial.println("STATE : IDLE");
    Serial.println("PRESS BUTTON");
}

void loop() {
    static unsigned long lastTime = millis();
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    if(dt > 0.1) dt = 0.1;

    switch(systemState) {
        case IDLE:
            digitalWrite(LED_PIN, LOW);
            if(buttonPressed()) startSystem();
            break;

        case STARTING:
            updateFastBlink();
            if(millis() - stateStartTime >= SYSTEM_SEQUENCE_TIME) {
                digitalWrite(LED_PIN, HIGH);
                systemState = RUNNING;
                systemStatus = "RUNNING";
                Serial.println("SYSTEM READY");
            }
            break;

        case RUNNING:
            if(buttonPressed()) {
                resetSystem();
                break;
            }
            readMasterArm();
            updateServos(dt);
            checkSave();
            break;

        case RESETTING:
            updateFastBlink();
            updateServos(dt);
            if(millis() - stateStartTime >= SYSTEM_SEQUENCE_TIME) {
                currentBase = targetBase = RESET_BASE;
                currentArm = targetArm = RESET_ARM;
                currentForearm = targetForearm = RESET_FOREARM;
                currentGripper = targetGripper = RESET_GRIPPER;

                servoBase.write(RESET_BASE);
                servoArm.write(RESET_ARM);
                servoForearm.write(RESET_FOREARM);
                servoGripper.write(RESET_GRIPPER);

                savePositions();
                digitalWrite(LED_PIN, LOW);
                systemState = IDLE;
                systemStatus = "IDLE";
                Serial.println("\nRESET COMPLETE\nSTATE IDLE");
            }
            break;
    }
    
    // Core 1 延时，保持控制循环的稳定性 (~100Hz)
    vTaskDelay(10 / portTICK_PERIOD_MS); 
}