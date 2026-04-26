import threading
import asyncio
import cv2
import time
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)
from ultralytics import YOLO

MAX_SPEED = 5.0      
YAW_SPEED = 60.0     
SMOOTH_FACTOR = 0.15 
GIMBAL_STEP = 10.0   

target_command = {"fwd": 0.0, "right": 0.0, "down": 0.0, "yaw": 0.0}
actual_command = {"fwd": 0.0, "right": 0.0, "down": 0.0, "yaw": 0.0}
target_gimbal = {"pitch": 0.0, "yaw": 0.0} 
telemetry_data = {"alt": 0.0} 

running = True
camera_busy = False
working_gimbal_id = 0 

async def move_camera(drone, pitch, yaw):
    global camera_busy, working_gimbal_id
    if camera_busy: return 
    camera_busy = True
    try:
        from mavsdk.gimbal import GimbalMode, SendMode
        await drone.gimbal.set_angles(
            working_gimbal_id, 0.0, pitch, yaw, GimbalMode.YAW_FOLLOW, SendMode.ONCE
        )
    except Exception: pass
    finally:
        await asyncio.sleep(0.1) 
        camera_busy = False

async def fetch_telemetry(drone):
    global telemetry_data
    async for position in drone.telemetry.position():
        telemetry_data["alt"] = position.relative_altitude_m

async def drone_logic():
    global running, target_gimbal, working_gimbal_id
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    async for state in drone.core.connection_state():
        if state.is_connected: break

    asyncio.create_task(fetch_telemetry(drone))

    from mavsdk.gimbal import ControlMode
    for gid in [1, 0, 2]: 
        try:
            await drone.gimbal.take_control(gid, ControlMode.PRIMARY)
            working_gimbal_id = gid
            break
        except: continue

    while True:
        try:
            await drone.action.arm()
            break
        except Exception:
            await asyncio.sleep(2)

    await drone.action.takeoff()
    await asyncio.sleep(5)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try: await drone.offboard.start()
    except OffboardError: return

    last_pitch, last_yaw = 0.0, 0.0
    while running:
        for axis in ["fwd", "right", "down", "yaw"]:
            diff = target_command[axis] - actual_command[axis]
            actual_command[axis] += diff * SMOOTH_FACTOR

        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(
            actual_command["fwd"], actual_command["right"], 
            actual_command["down"], actual_command["yaw"]
        ))

        if round(target_gimbal["pitch"], 1) != round(last_pitch, 1) or round(target_gimbal["yaw"], 1) != round(last_yaw, 1):
            asyncio.create_task(move_camera(drone, target_gimbal["pitch"], target_gimbal["yaw"]))
            last_pitch, last_yaw = target_gimbal["pitch"], target_gimbal["yaw"]

        await asyncio.sleep(0.05)

    await drone.action.land()

class VideoStream:
    def __init__(self):
        PIPE = "udpsrc port=5600 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false drop=true max-buffers=1"
        self.stream = cv2.VideoCapture(PIPE, cv2.CAP_GSTREAMER)
        self.frame = None
        self.running = True
        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while self.running:
            ret, frame = self.stream.read()
            if ret: self.frame = frame
            else: time.sleep(0.01)
            
    def read(self): return self.frame
    
    def stop(self):
        self.running = False
        self.stream.release()

def main():
    global running
    model = YOLO('yolov8s.pt') 
    threading.Thread(target=lambda: asyncio.run(drone_logic()), daemon=True).start()
    video = VideoStream()

    control_mode = 0 
    TARGET_CLASSES = [2, 7, 8] 
    
    KP_YAW = 0.12       
    KP_FWD = 0.00025    
    KP_PITCH = 0.06     
    TARGET_AREA = 15000 

    while running:
        frame = video.read()
        if frame is None: continue

        results = model(frame, verbose=False, conf=0.4)
        annotated_frame = results[0].plot()
        h, w = annotated_frame.shape[:2]

        cv2.line(annotated_frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 255, 0), 2)
        cv2.line(annotated_frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 255, 0), 2)

        target_locked = False

        if control_mode > 0: 
            best_target = None
            max_area = 0
            for box in results[0].boxes:
                if int(box.cls[0]) in TARGET_CLASSES:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        best_target = [x1, y1, x2, y2]

            if best_target:
                target_locked = True
                x1, y1, x2, y2 = best_target
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                err_x, err_y = cx - (w / 2), cy - (h / 2)
                
                target_command["yaw"] = err_x * KP_YAW
                target_gimbal["pitch"] = max(-90.0, min(0.0, target_gimbal["pitch"] - (err_y * KP_PITCH)))

                if control_mode == 2:
                    target_command["fwd"] = max(-MAX_SPEED, min(MAX_SPEED, (TARGET_AREA - max_area) * KP_FWD))

                cv2.drawMarker(annotated_frame, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
                cv2.line(annotated_frame, (w//2, h//2), (int(cx), int(cy)), (0, 0, 255), 1)
            else:
                target_command["yaw"] = 0.0
                if control_mode == 2: target_command["fwd"] = 0.0

        modes_text = ["MANUAL", "ASSISTED", "FULL AUTO"]
        colors = [(0, 255, 0), (0, 255, 255), (0, 0, 255)]
        
        cv2.putText(annotated_frame, f"MODE: {modes_text[control_mode]}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colors[control_mode], 2)
        status_text = "LOCKED" if target_locked else ("SEARCHING..." if control_mode > 0 else "STANDBY")
        status_color = (0, 0, 255) if target_locked else (0, 255, 0)
        cv2.putText(annotated_frame, f"TARGET: {status_text}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

        hud_x = w - 180
        cv2.putText(annotated_frame, f"ALT : {telemetry_data['alt']:.1f} m", (hud_x, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(annotated_frame, f"SPD : {actual_command['fwd']:.1f} m/s", (hud_x, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(annotated_frame, f"YAW : {actual_command['yaw']:.1f} d/s", (hud_x, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(annotated_frame, f"CAM : {target_gimbal['pitch']:.1f} deg", (hud_x, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Typhoon Hunter", annotated_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 27: running = False 
        
        elif key == ord('t'): 
            control_mode = (control_mode + 1) % 3 
            for k in target_command: target_command[k] = 0.0 

        elif key == ord(' '):
            if control_mode == 2:
                control_mode = 0 
            for k in target_command: target_command[k] = 0.0

        elif key == ord('r'): target_command["down"] = -1.5     
        elif key == ord('f'): target_command["down"] = 1.5      
        elif key == ord('g'): target_command["down"] = 0.0 

        if control_mode < 2:
            if key == ord('w'): target_command["fwd"] = MAX_SPEED
            elif key == ord('s'): target_command["fwd"] = -MAX_SPEED
            elif key == ord('a'): target_command["right"] = -MAX_SPEED
            elif key == ord('d'): target_command["right"] = MAX_SPEED
            
            if key == ord('q'): target_command["yaw"] = -YAW_SPEED
            elif key == ord('e'): target_command["yaw"] = YAW_SPEED
        
        if key == ord('i'): target_gimbal["pitch"] = max(-90.0, target_gimbal["pitch"] - GIMBAL_STEP)
        elif key == ord('k'): target_gimbal["pitch"] = min(0.0, target_gimbal["pitch"] + GIMBAL_STEP)
        if key == ord('j'): target_gimbal["yaw"] = max(-180.0, target_gimbal["yaw"] - GIMBAL_STEP)
        elif key == ord('l'): target_gimbal["yaw"] = min(180.0, target_gimbal["yaw"] + GIMBAL_STEP)

    video.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
