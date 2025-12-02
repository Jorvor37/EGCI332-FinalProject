import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import urllib.request
import time
import RPi.GPIO as GPIO
#from collections import deque
#import QRdetection.py

# --- DOOR 1 ---
ENA_PIN = 15
IN1_PIN = 16
IN2_PIN = 18

# --- DOOR 2 ---
ENB_PIN = 11
IN3_PIN = 40
IN4_PIN = 13

# --- DOOR 3 ---
ENC_PIN = 29
IN5_PIN = 7                       
IN6_PIN = 36

# --- CONVEYOR MOTOR ---
ENE_PIN = 33
IN9_PIN = 35
IN10_PIN = 37

# --- SET UP ---
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup([
    ENA_PIN, IN1_PIN, IN2_PIN,
    ENB_PIN, IN3_PIN, IN4_PIN,
    ENC_PIN, IN5_PIN, IN6_PIN,
    ENE_PIN, IN9_PIN, IN10_PIN
    ], GPIO.OUT)

pwmA = GPIO.PWM(ENA_PIN, 100)
pwmB = GPIO.PWM(ENB_PIN, 100)
pwmC = GPIO.PWM(ENC_PIN, 100)
pwmE = GPIO.PWM(ENE_PIN, 100)

# --- START ALL PWMs ---
pwmA.start(0)
pwmB.start(0)
pwmC.start(0)
pwmE.start(0)

# ======= MOTOR CONTROL =======
def ccw(a, b):
    GPIO.output(a, GPIO.HIGH)
    GPIO.output(b, GPIO.LOW)

def cw(a, b):
    GPIO.output(b, GPIO.HIGH)
    GPIO.output(a, GPIO.LOW)

def open_door(num_door, spd = 70, t=0.5):
    print(f"[INFO] OPEN DOOR {num_door}")
    if num_door == 1:
        ccw(IN1_PIN, IN2_PIN)
        pwmA.ChangeDutyCycle(spd)
        time.sleep(t)
        pwmA.ChangeDutyCycle(0)
    elif num_door == 2:
        ccw(IN3_PIN, IN4_PIN)
        pwmB.ChangeDutyCycle(spd)
        time.sleep(t)
        pwmB.ChangeDutyCycle(0)
    elif num_door == 3:
        ccw(IN5_PIN, IN6_PIN)
        pwmC.ChangeDutyCycle(spd)
        time.sleep(t)
        pwmC.ChangeDutyCycle(0)

def close_door(num_door, spd = 70, t=0.5):
    print(f"[INFO] CLOSE DOOR {num_door}")
    if num_door == 1:
        cw(IN1_PIN, IN2_PIN)
        pwmA.ChangeDutyCycle(spd)
        time.sleep(t)
        pwmA.ChangeDutyCycle(0)
    elif num_door == 2:
        cw(IN3_PIN, IN4_PIN)
        pwmB.ChangeDutyCycle(spd)
        time.sleep(t)
        pwmB.ChangeDutyCycle(0)
    elif num_door == 3:
        cw(IN5_PIN, IN6_PIN)
        pwmC.ChangeDutyCycle(spd)
        time.sleep(t)
        pwmC.ChangeDutyCycle(0)
    
def start_conveyor(spd = 60):
    cw(IN9_PIN, IN10_PIN)
    pwmE.ChangeDutyCycle(spd)

def stop_conveyor():
    pwmE.ChangeDutyCycle(0)
    
def closeAllDoors():
    close_door(1)
    close_door(2)
    close_door(3)

def stopAllDoors():
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    pwmC.ChangeDutyCycle(0)



# ====== POSTAL PREFIX ======
central = [                 # total: 25(+1)
    *list(range(10, 19)),   # 10-18: Bangkok + Central provinces
    *list(range(20, 28)),   # 20-27: Eastern region
    *list(range(70, 78)),   # 70-77: Western region
]

northern = [                # total: 17
    *list(range(50, 59)),   # 50-58: Upper Northern region
    *list(range(60, 68)),   # 60-67: Lower Northern region
]

northeast = [               # total: 20
    *list(range(30, 40)),   # 30-39: Lower Isan
    *list(range(40, 50)),   # 40-49: Upper Isan
]

southern = [                # total: 14
    *list(range(80, 87)),   # 80-86: Upper Southern region
    *list(range(90, 97)),   # 90-96: Lower Southern region
]

# ====== MAPPING ======
region_map = {}

for p in central:
    region_map[p] = 1 #"central"

for p in northern:
    region_map[p] = 2 #"northern"

for p in northeast:
    region_map[p] = 2 #"northeast"

for p in southern:
    region_map[p] = 3 #"southern"


def get_region(postal_code: str):
    if len(postal_code) != 5:
        return None
    if not postal_code.isdigit():
        return None
    
    prefix = int(postal_code[:2])
    return region_map.get(prefix, None)

# ====== MAIN ======
# Track conveyor state
conveyor_running = False


def ensure_conveyor_running():
    """Start conveyor only if not already moving."""
    global conveyor_running
    if not conveyor_running:
        start_conveyor(80)
        conveyor_running = True
        print("[CONVEYOR] Started")


def ensure_conveyor_stopped():
    """Stop conveyor only if no remaining parcels."""
    global conveyor_running
    if conveyor_running and len(events) == 0:
        stop_conveyor()
        conveyor_running = False
        print("[CONVEYOR] Stopped")
        
# ======  ESP32 CAMERA SETUP  ======

URL = "http://192.168.73.22/cam-lo.jpg"

cv2.namedWindow("ESP32-CAM", cv2.WINDOW_AUTOSIZE)
last_data = b""

try:
    postal_input = "00000"
    while True:
        frame = None
        # TRY TO FETCH ESP32 FRAME
        try:
            resp = urllib.request.urlopen(URL, timeout=2)
            img_bytes = resp.read()
            img_np = np.array(bytearray(img_bytes), dtype=np.uint8)
            frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
        except Exception as e:
            print("[WARNING] Failed to fetch frame:", e)
            frame = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(frame, "NO SIGNAL", (60,120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            time.sleep(0.2)
            
        # BARCODE SCANNING
        try:
            decoded = pyzbar.decode(frame)
            for obj in decoded:
                data = obj.data
                if data != last_data:
                    print("[QR DETECTED] Type:", obj.type, "Data:", data.decode("utf-8"))
                    last_data = data

                # Display text on frame
                cv2.putText(frame, data.decode("utf-8"), (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)
        except Exception as e:
            print("[WARNING] QR decode failed:", e)
            
        if last_data != b"":  # QR detected at least once
            cur_postal_input = last_data.decode("utf-8")
            door_id = get_region(cur_postal_input)
            #if cur_postal_input != postal_input:
            if door_id is None:
                print("[ERROR] Invalid postal → sending to end of conveyor")
                start_conveyor(80)
                time.sleep(2)
                stop_conveyor()
                
                #RESET FOR NEXT QR
                last_data = b""
                postal_input = "00000"
                print("[SYSTEM] Ready for next QR code")
            else:
                print(f"[SCHEDULE] Parcel → Door {door_id}")
                open_door(door_id)
                start_conveyor(80)
                time.sleep(door_id-0.4*door_id)
                close_door(door_id)
                stop_conveyor()
                
                #RESET FOR NEXT QR
                last_data = b""
                postal_input = "00000"
                print("[SYSTEM] Ready for next QR code")
            postal_input = cur_postal_input
            
        cv2.imshow("ESP32-CAM", frame)
        #if cv2.waitKey(1) == 27:
            #break
        time.sleep(0.02)

except KeyboardInterrupt:
    pass

finally:
    stopAllDoors()
    closeAllDoors()
    stop_conveyor()
    for pwm in (pwmA, pwmB, pwmC, pwmE):
        pwm.stop()
    GPIO.cleanup()
    print("[SYSTEM] Shutdown")


