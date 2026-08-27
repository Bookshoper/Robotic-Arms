import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import serial
import time
from collections import deque

# ================= 1. 配置串口 =================
# ⚠️ 请根据实际情况修改 'COM'，如果你没有连接机械臂，它会以测试模式运行
SERIAL_PORT = 'COM4' 
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"成功连接到串口 {SERIAL_PORT}")
except Exception as e:
    ser = None
    print(f"⚠️ 串口未连接，将以无硬件测试模式运行！(错误: {e})")

# ================= 2. 映射字典 =================
GESTURE_CMD_MAP = {
    'Open_Palm': 'S',   # 张开手掌 -> 待机/停止
    'Closed_Fist': 'G', # 握拳 -> 抓取
    'Pointing_Up': 'U', # 食指上举 -> 抬升
    'Victory': 'R',     # V字手势 -> 旋转
    'None': 'S'         # 未识别到手 -> 待机/停止
}

# ================= 3. 初始化 MediaPipe =================
base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
options = vision.GestureRecognizerOptions(base_options=base_options)
recognizer = vision.GestureRecognizer.create_from_options(options)

# ================= 4. 初始化摄像头与防抖 =================
cap = cv2.VideoCapture(0) # 0通常是笔记本自带摄像头
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 用于稳定性判断，保存最近5帧的手势结果
STABILITY_FRAMES = 5
gesture_history = deque(maxlen=STABILITY_FRAMES)
last_sent_cmd = 'S' # 记录上次发送的指令，避免重复发送

print("启动摄像头... 按下 'q' 键退出程序。")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
        
    # 镜像翻转画面，符合照镜子的直觉
    frame = cv2.flip(frame, 1)
    
    # 转换颜色空间：OpenCV 默认 BGR，MediaPipe 需要 RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

    # 进行手势识别
    recognition_result = recognizer.recognize(mp_image)
    
    current_gesture = "None"
    
    # 解析识别结果
    if recognition_result.gestures:
        # 获取得分最高的手势名称
        top_gesture = recognition_result.gestures[0][0].category_name
        if top_gesture in GESTURE_CMD_MAP:
            current_gesture = top_gesture
            
            # 画出手的关键点连线（仅做视觉反馈）
            if recognition_result.hand_landmarks:
                landmarks = recognition_result.hand_landmarks[0]
                for mark in landmarks:
                    x = int(mark.x * frame.shape[1])
                    y = int(mark.y * frame.shape[0])
                    cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)

    # --- 核心逻辑：稳定性判断 ---
    gesture_history.append(current_gesture)
    target_cmd = 'S' # 默认安全停止指令
    
    # 如果没手，立刻强制切断/停止 (安全第一)
    if current_gesture == "None":
        target_cmd = 'S'
        stable_gesture_name = "None (Stop)"
    # 如果最近5帧识别结果一模一样，说明手势稳定了
    elif len(gesture_history) == STABILITY_FRAMES and len(set(gesture_history)) == 1:
        stable_gesture = gesture_history[0]
        target_cmd = GESTURE_CMD_MAP[stable_gesture]
        stable_gesture_name = stable_gesture
    else:
        # 手势在变化中，维持上一个指令不变，等待稳定
        target_cmd = last_sent_cmd
        stable_gesture_name = "Wait for stability..."

    # --- 发送串口指令 ---
    if target_cmd != last_sent_cmd:
        if ser and ser.is_open:
            ser.write(target_cmd.encode('utf-8'))
        print(f"发送动作指令: {target_cmd}")
        last_sent_cmd = target_cmd

    # ================= 5. 可视化界面 (UI) =================
    # 在屏幕左上角显示当前信息
    cv2.putText(frame, f"Raw Gesture: {current_gesture}", (10, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                
    # 显示稳定后的指令
    color = (0, 255, 0) if target_cmd != 'S' else (0, 0, 255)
    cv2.putText(frame, f"Command: {target_cmd}", (10, 80), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    cv2.imshow('Gesture Control Window', frame)

    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
if ser:
    ser.write('S'.encode('utf-8')) # 退出前确保机械臂停止
    ser.close()