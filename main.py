import time
import RPi.GPIO as GPIO
from collections import deque
from qr_scanner import QRScanner

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
        cw(IN3_PIN, IN4_PIN)
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
        ccw(IN3_PIN, IN4_PIN)
        pwmB.ChangeDutyCycle(spd)
        time.sleep(t)
        pwmB.ChangeDutyCycle(0)
    elif num_door == 3:
        cw(IN5_PIN, IN6_PIN)
        pwmC.ChangeDutyCycle(spd)
        time.sleep(t)
        pwmC.ChangeDutyCycle(0)
    
def start_conveyor(spd = 60):
    ccw(IN9_PIN, IN10_PIN)
    pwmE.ChangeDutyCycle(spd)

def stop_conveyor():
    pwmE.ChangeDutyCycle(0)

def stopAllDoors():
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    pwmC.ChangeDutyCycle(0)

def closeAllDoors():
    close_door(1)
    close_door(2)
    close_door(3)

# ====== POSTAL PREFIX ======
central = [                                     # total: 25(+1)
    *list(range(10, 19)),                       # 10-18: Bangkok + Central provinces
    *list(range(20, 28)),                       # 20-27: Eastern region
    *list(range(70, 78)),                       # 70-77: Western region
]

northern = [                                    # total: 17
    *list(range(50, 59)),                       # 50-58: Upper Northern region
    *list(range(60, 68)),                       # 60-67: Lower Northern region
]

northeast = [                                   # total: 20
    *list(range(30, 40)),                       # 30-39: Lower Isan
    *list(range(40, 50)),                       # 40-49: Upper Isan
]

southern = [                                    # total: 14
    *list(range(80, 87)),                       # 80-86: Upper Southern region
    *list(range(90, 97)),                       # 90-96: Lower Southern region
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


# ================= QR CALLBACK =================
latest_postal = None

def on_qr(data):
    global latest_postal
    latest_postal = data.strip()
    print(f"[QR] Detected: {latest_postal}")

# ====== MAIN ======
# how long it takes to reach the door (unreadable = door 0)
travel_time = {
    1: 1,  # central
    2: 2,  # northern
    3: 2,  # northeast
    4: 3,  # southern
    0: 5   # unreadable → end of conveyor
}
door_open_lead = 0.3  # seconds before arrival to open door
events = deque()  # Event queue: (event_time, action, door_id)
conveyor_running = False  # Track conveyor state

scanner = QRScanner("http://192.168.1.55/cam-lo.jpg")
scanner.start(on_qr)
print("[SYSTEM] QR Scanner Running...")

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

try:
    # start_conveyor(80)
    # print("[SYSTEM] Conveyor Running.")

    while True:
        # Replace with scanner input
        #postal_input = "10160SAD"#input("Scan Parcel: ").strip()

        if latest_postal is None:
            time.sleep(0.02)
            continue  # no QR scanned yet, skip loop

        postal_input = latest_postal
        latest_postal = None  # consume once

        door_id = get_region(postal_input)
        if door_id is None:
            print("[ERROR] Invalid postal → sending to end of conveyor")
            door_id = 0

        now = time.time()
        arrival_time = now + travel_time[door_id]

        # Schedule open/close events
        events.append((arrival_time - door_open_lead, "open", door_id))
        events.append((arrival_time, "close", door_id))

        print(f"[SCHEDULE] Parcel → Door {door_id}, ETA: {arrival_time:.2f}")

        # Ensure conveyor is ON when there is at least one parcel
        ensure_conveyor_running()

        # Check events
        now = time.time()
        for _ in range(len(events)):
            event_time, action, door = events[0]
            if now >= event_time:
                events.popleft()

                if action == "open": 
                    open_door(door)
                elif action == "close": 
                    close_door(door)
            else:
                break
    
        # If no more scheduled events → stop conveyor
        ensure_conveyor_stopped()
        time.sleep(0.02)  # loop delay

except KeyboardInterrupt:
    pass

finally:
    closeAllDoors()
    stopAllDoors()
    stop_conveyor()
    for pwm in (pwmA, pwmB, pwmC, pwmE):
        pwm.stop()
    GPIO.cleanup()
    print("[SYSTEM] Shutdown")
