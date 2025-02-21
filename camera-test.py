import cv2
import RPi.GPIO as GPIO
import time
import threading
import numpy as np

# Motor Configuration
LEFT_MOTOR = 5  # GPIO pin for motor control
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_MOTOR, GPIO.OUT)

# Setup PWM for motor control
left_pwm = GPIO.PWM(LEFT_MOTOR, 100)  # 100Hz frequency
left_pwm.start(0)  # Start with motor off

# Global flags
stop_program = False  # Stops the program manually
motor_running = True  # Motor state

# Function to monitor user input for stopping the program
def monitor_input():
    global stop_program, motor_running
    while True:
        user_input = input("Press 'q' to stop the program: ").strip().lower()
        if user_input == 'q':
            print("Manual stop triggered. Stopping motor...")
            stop_program = True
            motor_running = False
            left_pwm.ChangeDutyCycle(0)  # Immediately stop motor
            break

# Function to process the frame and create the combined output
def process_and_combine(frame, frame_count):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Painter's blue tape HSV range
    lower_blue = (100, 80, 50)
    upper_blue = (130, 255, 255)

    # Create the initial HSV mask
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Clean the mask using morphological operations
    kernel = np.ones((5, 5), np.uint8)
    cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_CLOSE, kernel)

    # Combine images: Original, HSV Mask, and Cleaned Mask
    original_resized = cv2.resize(frame, (300, 300))
    mask_resized = cv2.resize(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), (300, 300))
    cleaned_resized = cv2.resize(cv2.cvtColor(cleaned_mask, cv2.COLOR_GRAY2BGR), (300, 300))

    combined = cv2.hconcat([original_resized, mask_resized, cleaned_resized])

    # Save the combined output
    output_path = f"combined_output_{frame_count}.jpg"
    cv2.imwrite(output_path, combined)
    print(f"Saved combined output as '{output_path}'.")

    # Calculate blue percentage
    blue_pixels = cv2.countNonZero(cleaned_mask)
    total_pixels = cleaned_mask.shape[0] * cleaned_mask.shape[1]
    blue_percentage = (blue_pixels / total_pixels) * 100

    return blue_percentage

# Main function
def main():
    global stop_program, motor_running

    # Initialize camera
    cap = cv2.VideoCapture("libcamerasrc ! video/x-raw, width=640, height=480 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Error: Camera not detected.")
        return

    print("Starting motor. Looking for blue tape...")
    left_pwm.ChangeDutyCycle(60)  # Start motor forward

    # Start input monitoring thread
    input_thread = threading.Thread(target=monitor_input)
    input_thread.daemon = True
    input_thread.start()

    frame_count = 0  # Counter to track saved frames

    try:
        while motor_running and not stop_program:
            # Capture frame from camera
            ret, frame = cap.read()
            if not ret:
                print("Error: Unable to capture frame.")
                continue

            # Process the frame and save combined output
            frame_count += 1
            blue_percentage = process_and_combine(frame, frame_count)
            print(f"Blue percentage: {blue_percentage:.2f}%")

            # Stop motor if blue is detected
            if blue_percentage > 5:  # Threshold for blue detection
                print("Blue tape detected! Stopping motor...")
                left_pwm.ChangeDutyCycle(0)  # Stop motor
                motor_running = False
                break

            time.sleep(0.5)  # Check every 0.5 seconds

    except KeyboardInterrupt:
        print("Program interrupted by user.")

    finally:
        # Cleanup
        cap.release()
        left_pwm.ChangeDutyCycle(0)
        left_pwm.stop()
        GPIO.cleanup()
        print("Cleaned up GPIO and exited.")

if __name__ == "__main__":
    main()
