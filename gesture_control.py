import cv2
import time
import serial
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# ================= 1. 串口配置 =================
SERIAL_PORT = 'COM4'  # ⚠️ 请确保这里和你的 ESP32 端口一致
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"成功连接串口 {SERIAL_PORT}")
except Exception as e:
    ser = None
    print(f"⚠️ 串口未连接，处于无硬件测试模式: {e}")

# ================= 2. 初始化 Gesture Recognizer Task 模型 =================
base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
options = vision.GestureRecognizerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.IMAGE,
    num_hands=1
)
recognizer = vision.GestureRecognizer.create_from_options(options)

# ================= 3. 防抖与延迟检测参数 =================
cap = cv2.VideoCapture(0)

last_sent_cmd = None         # 记录最终发送给 ESP32 的指令
candidate_cmd = None         # 当前正在检测的“候选”指令
candidate_start_time = 0     # 候选指令开始计时的时间
STABILITY_TIME = 0.5         # 延迟检测时间 (0.5秒 = 500ms)

print("系统启动：带 500ms 防抖检测 & 瞬间急停机制")
print("张开=开爪 | 握拳=握爪 | 1指=下 | 2指=上 | 3指=左转 | 4指=右转 | 无手势=急停(S)")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
        
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
    recognition_result = recognizer.recognize(mp_image)

    raw_cmd = None
    base_cmd_text = ""

    # ================= 4. 识别手势 =================
    if recognition_result.gestures and recognition_result.hand_landmarks:
        gesture_category = recognition_result.gestures[0][0].category_name
        landmarks = recognition_result.hand_landmarks[0]

        h, w, _ = frame.shape
        for mark in landmarks:
            cx, cy = int(mark.x * w), int(mark.y * h)
            cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)

        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        up_count = sum([1 for tip, pip in zip(tips, pips) if landmarks[tip].y < landmarks[pip].y])

        if gesture_category == "Closed_Fist":
            raw_cmd, base_cmd_text = 'O', "Fist -> Close (握爪)"
        elif gesture_category == "Open_Palm":
            raw_cmd, base_cmd_text = 'C', "Open -> Open (开爪)"
        elif up_count == 1:
            raw_cmd, base_cmd_text = 'U', "1 Finger -> Down (向下)"
        elif up_count == 2:
            raw_cmd, base_cmd_text = 'D', "2 Fingers -> Up (向上)"
        elif up_count == 3:
            raw_cmd, base_cmd_text = 'R', "3 Fingers -> Left (左转)"
        elif up_count == 4:
            raw_cmd, base_cmd_text = 'L', "4 Fingers -> Right (右转)"

    # ================= 5. 500ms 稳定性检测 & 急停发送逻辑 =================
    current_time = time.time()

    if raw_cmd:
        # 有手势时，走 500ms 延迟确认逻辑
        if raw_cmd != candidate_cmd:
            candidate_cmd = raw_cmd                  # 记录新动作
            candidate_start_time = current_time      # 重新开始计时
            display_text = f"{base_cmd_text} (Detecting...)"
        else:
            elapsed_time = current_time - candidate_start_time
            if elapsed_time >= STABILITY_TIME:
                display_text = f"{base_cmd_text} (Stable!)"
                if candidate_cmd != last_sent_cmd:
                    if ser and ser.is_open:
                        ser.write(candidate_cmd.encode('utf-8'))
                        print(f"✅ 发送稳定指令: {candidate_cmd} ({base_cmd_text})")
                    last_sent_cmd = candidate_cmd  
            else:
                countdown = STABILITY_TIME - elapsed_time
                display_text = f"{base_cmd_text} (Wait {countdown:.1f}s)"
    else:
        # ⚠️ 没有有效手势时，瞬间触发急停逻辑 (绕过 500ms 倒计时)
        candidate_cmd = None
        display_text = "No Hand -> STOP (急停)"
        
        # 确保不会重复疯狂发 'S'
        if last_sent_cmd != 'S':
            if ser and ser.is_open:
                ser.write('S'.encode('utf-8'))
                print("🛑 丢失手势，瞬间发送急停指令: S")
            last_sent_cmd = 'S'

    # ================= 6. UI 显示 =================
    if candidate_cmd and (candidate_cmd == last_sent_cmd):
        color = (0, 255, 0)   # 绿 (已稳定并发送动作)
    elif candidate_cmd:
        color = (0, 255, 255) # 黄 (倒计时中)
    elif last_sent_cmd == 'S':
        color = (0, 0, 255)   # 红 (急停状态)
    else:
        color = (255, 255, 255) 

    cv2.putText(frame, display_text, (10, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.imshow('Gesture Control (500ms Delay + Auto Stop)', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if ser:
    ser.close()
