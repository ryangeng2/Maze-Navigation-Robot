import time
import utime
from machine import Pin, PWM, time_pulse_us, I2C
from pico_i2c_lcd import I2cLcd

#Pins

#i2c LCD setup
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)
I2C_ADDR = i2c.scan()[0]
lcd = I2cLcd(i2c, I2C_ADDR, 2, 16)

# Motor control pins 
motor1pin1 = Pin(26, Pin.OUT)  # Motor 1 IN1 (LEFT)
motor1pin2 = Pin(27, Pin.OUT)  # Motor 1 IN2 (LEFT)
motor2pin1 = Pin(22, Pin.OUT)  # Motor 2 IN1 (RIGHT)
motor2pin2 = Pin(15, Pin.OUT)  # Motor 2 IN2 (RIGHT)

# PWM Pins
ENA = PWM(Pin(9))   # Motor 1 PWM (LEFT)
ENB = PWM(Pin(10))  # Motor 2 PWM (RIGHT)
ENA.freq(1000)
ENB.freq(1000)

# Ultrasonic sensor 1 (FRONT)
trigPin1 = Pin(0, Pin.OUT)
echoPin1 = Pin(1, Pin.IN)

# Ultrasonic sensor 2 (RIGHT)
trigPin2 = Pin(12, Pin.OUT)
echoPin2 = Pin(13, Pin.IN)

# Ultrasonic sensor 3 (LEFT)
trigPin3 = Pin(2, Pin.OUT)
echoPin3 = Pin(3, Pin.IN)

#Robot global variables, used for tuning. 

# Calibration offsets
wheelOffR = 1.0
wheelOffL = 0.9

BASE_SPEED = 52000    
MAX_DUTY   = 60000

TARGET_DIST = 9        # Desired distance to wall when following a single side
FRONT_THRESHOLD = 11.0    # obstacle threshold straight ahead 
MAX_WALL_DIST = 30.0      # Max distance to consider "there is a wall" 

Kp_center = 1500       # Steerinig sensitivity when following two walls
Kp_side   = 1500          # steering sensitivity when following one wall

TURN_TIME_RIGHT = 450     # ms to turn approx 90° right
TURN_TIME_LEFT  = 417     # ms to turn approx 90° left
TURN_TIME_AROUND = 0   # ms to turn approx 180°


# Functions
def calculate_distance(trigPin, echoPin):
    trigPin.low()
    time.sleep_us(2)
    trigPin.high()
    time.sleep_us(10)
    trigPin.low()

    pulse_duration = time_pulse_us(echoPin, 1, 30000)  # 30 ms timeout

    if pulse_duration < 0:
        return None

    distance = pulse_duration * 0.034 / 2.0
    return distance

def safe_distance(trigPin, echoPin, default=200.0):
    d = calculate_distance(trigPin, echoPin)
    if d is None:
        return default
    return d

def set_motor_pwm(left_pwm, right_pwm):
    left_val  = int(max(0, min(MAX_DUTY, left_pwm * wheelOffL)))
    right_val = int(max(0, min(MAX_DUTY, right_pwm * wheelOffR)))
    ENA.duty_u16(left_val)
    ENB.duty_u16(right_val)

def drive_forward(speed, steering=0):

    left_pwm  = speed - steering
    right_pwm = speed + steering

    # Set direction forward
    motor1pin1.value(0)
    motor1pin2.value(1)
    motor2pin1.value(0)
    motor2pin2.value(1)

    set_motor_pwm(left_pwm, right_pwm)

def turn_left_in_place(speed, time_ms):
    # Left wheel backward, right wheel forward
    motor1pin1.value(1)
    motor1pin2.value(0)
    motor2pin1.value(0)
    motor2pin2.value(1)
    set_motor_pwm(speed, speed)
    time.sleep_ms(time_ms)
    stop_motors()

def turn_right_in_place(speed, time_ms):
    # Left wheel forward, right wheel backward
    motor1pin1.value(0)
    motor1pin2.value(1)
    motor2pin1.value(1)
    motor2pin2.value(0)
    set_motor_pwm(speed, speed)
    time.sleep_ms(time_ms)
    stop_motors()

def stop_motors():
    set_motor_pwm(0, 0)
    motor1pin1.value(0)
    motor1pin2.value(0)
    motor2pin1.value(0)
    motor2pin2.value(0)
    
        
def display_message(lcd, turn_direction, sens1, sens2, sens3):
    lcd.clear()
    lcd.putstr(f"Turning {turn_direction}")
    time.sleep(1)
    lcd.clear()
    lcd.putstr(f"S1: {sens1}, S2: {sens2}")
    lcd.move_to(0,1)
    lcd.putstr(f"S3: {sens3}")
    time.sleep(1)
    lcd.clear()

# Main loop (maze navigation)

while True:
    # Read distances
    sens1 = safe_distance(trigPin1, echoPin1)  # FRONT
    sens2 = safe_distance(trigPin2, echoPin2)  # RIGHT
    sens3 = safe_distance(trigPin3, echoPin3)  # LEFT
    
    roundS1 = round(sens1, 2)
    roundS2 = round(sens2, 2)
    roundS3 = round(sens3, 2)

    # Debug prints
    print("Front:", sens1, " Right:", sens2, " Left:", sens3)

    # wall deteceted ahead
    if sens1 < FRONT_THRESHOLD:
        # Something directly in front
        stop_motors()
        time.sleep_ms(500)
        
        right_open = sens2 > sens3
        left_open  = sens3 > sens2

        #displays appropriate error messages 
        if right_open:
            display_message(lcd, "Right", roundS1, roundS2, roundS3)
            turn_right_in_place(BASE_SPEED+2000, TURN_TIME_RIGHT)
            
        elif left_open:
            display_message(lcd, "Left", roundS1, roundS2, roundS3)
            turn_left_in_place(BASE_SPEED+2000, TURN_TIME_LEFT)
        time.sleep_ms(500)

        # After turning, loop back and re-scan
        continue

    # The centering and wall following logic
    steering = 0

    right_has_wall = sens2 < MAX_WALL_DIST
    left_has_wall  = sens3 < MAX_WALL_DIST

    if right_has_wall and left_has_wall:
        # center between two walls
        # If left > right we are closer to right -> steer left
        error = (sens3 - sens2) 
        steering = Kp_center * error

    elif right_has_wall and not left_has_wall:
        # Follow right wall at TARGET_DIST
        # if sens2 < target (too close), error positive -> steer left
        #keeps it from veering off when no walls are detected.
        error = (TARGET_DIST - sens2) 
        steering = Kp_side * error 

    elif left_has_wall and not right_has_wall:
        # Follow left wall at TARGET_DIST
        # If sens3 < target (too close), then steer right (negative steering)
        error = (sens3 - TARGET_DIST)
        steering = Kp_side * error 
    else:
        stop_motors()
        time.sleep(3000)
        # No walls detected nearby -> end of maze reached

    # Drive forward with steering correction
    print(steering)
    drive_forward(BASE_SPEED, steering)

    # delay to slow down robot and to prevent sensor overload
    time.sleep_ms(20)
    stop_motors()