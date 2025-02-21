import cv2
import RPi.GPIO as GPIO
import time
import threading
import smbus
import numpy as np
import os

# --- Motor Configuration ---
LEFT_MOTOR = 5  # GPIO pin for motor control
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_MOTOR, GPIO.OUT)

# Setup PWM for motor control at 30Hz
left_pwm = GPIO.PWM(LEFT_MOTOR, 100)  # 30Hz frequency for slow forward
left_pwm.start(0)  # Start with motor off

# --- I2C Configuration for Arduino ---
ARDUINO_ADDRESS = 0x08
bus = smbus.SMBus(1)  # I2C bus 1 on Raspberry Pi

# Servo positions
SERVO_START = 136
SERVO_RIGHT = 39
INITIAL_START = 136


# Ultrasonic Sensor 1 (Raspberry Pi) Pins
TRIG_PIN = 27  # GPIO pin for Trig
ECHO_PIN = 22  # GPIO pin for Echo
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Global flags
stop_program = False
motor_running = True

# Path to save video
save_path = "/home/pi/new/photos/segment.avi"

# Threshold distance in cm
STOP_DISTANCE = 12


def send_servo_position(position):
    """
    Sends a position value (0-180) to the Arduino via I2C.
    """
    try:
        bus.write_byte(ARDUINO_ADDRESS, position)
        print(f"Sent servo position: {position}")
    except Exception as e:
        print(f"Error sending position: {e}")


def ultrasonic_distance():
    """
    Measures distance using the Raspberry Pi's ultrasonic sensor.
    """
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.05)

    # Send 10us pulse to trigger
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Measure the time for echo
    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        pulse_start = time.time()

    pulse_end = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = round(pulse_duration * 17150, 2)  # Convert to cm
    return distance


def monitor_input():
    """
    Monitors keyboard input to stop the program manually.
    """
    global stop_program, motor_running
    while True:
        user_input = input("Press 'q' to stop the program: ").strip().lower()
        if user_input == 'q':
            print("Manual stop triggered. Stopping motor...")
            stop_program = True
            motor_running = False
            left_pwm.ChangeDutyCycle(0)  # Immediately stop motor
            break


def detect_blue(frame):
    """
    Detects blue in the frame and returns True if blue percentage exceeds the threshold.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = (100, 80, 50)
    upper_blue = (130, 255, 255)

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_pixels = cv2.countNonZero(mask)
    total_pixels = mask.shape[0] * mask.shape[1]
    blue_percentage = (blue_pixels / total_pixels) * 100

    return blue_percentage > 5  # Threshold for blue detection


def parking_sequence():
    """
    Perform the parking sequence:
    1. Turn servo right.
    2. Drive forward slowly while monitoring ultrasonic sensor to avoid obstacles.
    3. Return servo to center position.
    """
    print("Initiating parking sequence...")

    # Turn right
    send_servo_position(SERVO_RIGHT)
    time.sleep(1)

    print("Driving forward slowly...")
    left_pwm.ChangeDutyCycle(30)  # Slow forward speed

    try:
        while True:
            # Measure distance using ultrasonic sensor
            distance = ultrasonic_distance()
            print(f"Ultrasonic Sensor Distance: {distance} cm")

            # Stop motor if obstacle is detected
            if distance <= STOP_DISTANCE:
                print("Obstacle detected within 8cm! Stopping motor...")
                left_pwm.ChangeDutyCycle(0)
                break

            time.sleep(0.1)  # Small delay for distance measurement

    except KeyboardInterrupt:
        print("Parking sequence interrupted.")

    finally:
        # Stop motor and return servo to start position
        left_pwm.ChangeDutyCycle(0)
        send_servo_position(INITIAL_START)
        print("Servo returned to start position.")
        print("Parking sequence completed.")


def main():
    """
    Main function to control the motor, process camera frames, and save the video.
    """
    global stop_program, motor_running

    # Initialize camera using GStreamer
    cap = cv2.VideoCapture("libcamerasrc ! video/x-raw, width=640, height=480 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Error: Camera not detected.")
        return

    # Ensure save directory exists
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    # VideoWriter configuration
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(save_path, fourcc, 20.0, (640, 480))

    print("Starting motor. Recording and looking for blue...")
    send_servo_position(INITIAL_START)
    time.sleep(1)
    left_pwm.ChangeDutyCycle(70)  # Start motor forward at 60% speed

    # Start input monitoring thread
    input_thread = threading.Thread(target=monitor_input)
    input_thread.daemon = True
    input_thread.start()

    try:
        while motor_running and not stop_program:
            # Capture frame from camera
            ret, frame = cap.read()
            if not ret:
                print("Error: Unable to capture frame.")
                continue

            # Write frame to video
            out.write(frame)

            # Check for blue
            if detect_blue(frame):
                print("Blue detected! Stopping motor...")
                left_pwm.ChangeDutyCycle(0)  # Stop motor
                motor_running = False
                parking_sequence()  # Start parking sequence
                break

            time.sleep(0.1)  # Check every 0.1 seconds

    except KeyboardInterrupt:
        print("Program interrupted.")

    finally:
        # Cleanup
        cap.release()
        out.release()
        left_pwm.ChangeDutyCycle(0)
        left_pwm.stop()
        GPIO.cleanup()
        print(f"Video saved at {save_path}")
        print("Cleaned up GPIO and exited.")


if __name__ == "__main__":
    main()
