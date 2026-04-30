import queue
import json
import time
import sounddevice as sd
import serial
import ydlidar
import math
from vosk import Model, KaldiRecognizer

# ── Arduino ───────────────────────────────────────────────────────
try:
    arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(2)
    print("Arduino connected")
except serial.SerialException as e:
    arduino = None
    print(f"Arduino not found: {e}")

def send(cmd: str):
    if arduino:
        arduino.write((cmd + '\n').encode())
        arduino.flush()

# ── LiDAR ─────────────────────────────────────────────────────────
ydlidar.os_init()
laser = ydlidar.CYdLidar()

laser.setlidaropt(ydlidar.LidarPropSerialPort,     "/dev/ttyUSB0")
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
laser.setlidaropt(ydlidar.LidarPropSingleChannel,  False)
laser.setlidaropt(ydlidar.LidarPropLidarType,      ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType,     ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency,  10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate,     4)

assert laser.initialize(), "LiDAR init failed"
assert laser.turnOn(),     "LiDAR turnOn failed"

scan = ydlidar.LaserScan()

# ── LiDAR geometry ────────────────────────────────────────────────
FORWARD_OFFSET   = -27
OBSTACLE_DIST_MM = 300
SIDE_OBSTACLE_MM = 250

def normalize(angle_deg):
    a = angle_deg - FORWARD_OFFSET
    if a >  180: a -= 360
    if a < -180: a += 360
    return a

def get_sector_min(points, angle_min_deg, angle_max_deg, max_range=3000):
    min_d = max_range
    for p in points:
        angle_deg = normalize(math.degrees(p.angle))
        if angle_min_deg <= angle_deg <= angle_max_deg:
            dist = p.range * 1000
            if 50 < dist < max_range:
                min_d = min(min_d, dist)
    return min_d

def read_obstacles():
    if not laser.doProcessSimple(scan):
        return None, None, None
    pts   = scan.points
    front = get_sector_min(pts, -30,   30)
    left  = get_sector_min(pts,  31,  120)
    right = get_sector_min(pts, -120, -31)
    return front, left, right

# Autopilot state machine 
nav_state  = "IDLE"
state_start = time.time()

def elapsed():
    return time.time() - state_start

def change_state(new_state, cmd=None):
    global nav_state, state_start
    nav_state   = new_state
    state_start = time.time()
    if cmd:
        send(cmd)
    print(f"Nav state: {new_state}  cmd: {cmd}")

def navigate():
    front, left, right = read_obstacles()
    if front is None:
        return

    obs_front = front < OBSTACLE_DIST_MM
    obs_left  = left  < SIDE_OBSTACLE_MM
    obs_right = right < SIDE_OBSTACLE_MM

    if nav_state == "TURNING_LEFT":
        if elapsed() > 0.5:
            change_state("DRIVE_FORWARD", "FORWARD")
        return

    if nav_state == "TURNING_RIGHT":
        if elapsed() > 0.5:
            change_state("DRIVE_FORWARD", "FORWARD")
        return

    if nav_state == "BACKING_UP":
        if elapsed() > 0.3:
            if left > right:
                change_state("TURNING_LEFT",  "LEFT")
            else:
                change_state("TURNING_RIGHT", "RIGHT")
        return

    if nav_state == "TURN_AROUND":
        if elapsed() > 1.4:
            change_state("DRIVE_FORWARD", "FORWARD")
        return

    if not obs_front:
        if nav_state != "DRIVE_FORWARD":
            change_state("DRIVE_FORWARD", "FORWARD")
    elif obs_front and obs_left and obs_right:
        change_state("TURN_AROUND", "LEFT")
    elif obs_front and not obs_left:
        change_state("TURNING_LEFT",  "LEFT")
    elif obs_front and not obs_right:
        change_state("TURNING_RIGHT", "RIGHT")
    else:
        change_state("BACKING_UP", "BACKWARD")

# Robot mode
# mode = None      idle/stopped
# mode = "AUTO"    autopilot
# mode = "MANUAL"  manual voice commands

mode = None

def start_autopilot():
    global mode
    mode = "AUTO"
    change_state("DRIVE_FORWARD", "FORWARD")
    print("=== AUTOPILOT STARTED ===")

def start_manual():
    global mode
    mode = "MANUAL"
    send("STOP")
    print("=== MANUAL MODE - say: forward, backward, left, right, stop ===")

def stop_robot():
    global mode
    mode = None
    change_state("IDLE", "STOP")
    print("=== STOPPED ===")

def handle_manual_command(text):
    if   text == "forward":          send("FORWARD")
    elif text == "backward":         send("BACKWARD")
    elif text in ("left", "turn left"):   send("LEFT")
    elif text in ("right", "turn right"): send("RIGHT")
    elif text == "backward left":    send("BACKWARD_LEFT")
    elif text == "backward right":   send("BACKWARD_RIGHT")
    elif text == "stop":             send("STOP")
    else:
        print(f"Unknown manual command: '{text}'")

# ── Voice recognition ─────────────────────────────────────────────
audio_queue = queue.Queue()
model       = Model("/home/lilllo1717/projects/robotCar/vosk-model-small-en-us-0.15")

# all commands we want to recognise
vocab = [
    "start autopilot",
    "start manual",
    "stop",
    "forward",
    "backward",
    "left",
    "right",
    "turn left",
    "turn right",
    "backward left",
    "backward right",
    "[unk]"
]
recognizer = KaldiRecognizer(model, 16000, json.dumps(vocab))

def audio_callback(indata, frames, time_info, status):
    audio_queue.put(bytes(indata))

def handle_voice(text):
    print(f"Heard: '{text}'  mode: {mode}")

    # mode switching — always available
    if text == "start autopilot":
        start_autopilot()
        return
    if text == "start manual":
        start_manual()
        return
    if text == "stop":
        stop_robot()
        return

    # manual commands — only in manual mode
    if mode == "MANUAL":
        handle_manual_command(text)

# ── Main loop ─────────────────────────────────────────────────────
print("Ready. Say 'start autopilot' or 'start manual'.")

try:
    with sd.RawInputStream(samplerate=16000, blocksize=8000,
                           dtype="int16", channels=1,
                           callback=audio_callback):
        while True:
            # voice
            try:
                data = audio_queue.get_nowait()
                if recognizer.AcceptWaveform(data):
                    text = json.loads(recognizer.Result()).get("text", "").lower().strip()
                    if text:
                        handle_voice(text)
            except queue.Empty:
                pass

            # autopilot loop
            if mode == "AUTO":
                navigate()

except KeyboardInterrupt:
    print("Shutting down...")
finally:
    send("STOP")
    laser.turnOff()
    laser.disconnecting()
    if arduino:
        arduino.close()