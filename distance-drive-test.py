import smbus2
import time
import RPi.GPIO as GPIO

# I2C Setup
ARDUINO_ADDR = 0x08  # Arduino I2C Slave Address
bus = smbus2.SMBus(1)

# GPIO Configuration for Motor Control
LEFT_MOTOR = 5  # GPIO pin for DC motor PWM
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_MOTOR, GPIO.OUT)

# Setup PWM for motor control
left_pwm = GPIO.PWM(LEFT_MOTOR, 100)  # 100Hz frequency
left_pwm.start(0)  # Start with motor off

# Function to get ultrasonic sensor data from Arduino
def get_sensor_data():
    try:
        # Request 1 byte from Arduino (distance value from Sensor 3)
        data = bus.read_byte(ARDUINO_ADDR)
        return data  # Distance in cm
    except Exception as e:
        print(f"Error reading I2C data: {e}")
        return None

# Main Loop to Drive Motor and Stop Based on Distance
try:
    print("Starting motor control. Press CTRL+C to stop.")
    motor_running = True  # State flag for the motor
    
    while motor_running:
        distance = get_sensor_data()
        if distance is not None:
            print(f"Distance from Sensor 3: {distance} cm")
            
            if distance > 10:  # Safe distance threshold (10 cm)
                left_pwm.ChangeDutyCycle(60)  # Drive motor at 60% speed
                print("Motor running forward...")
            else:
                left_pwm.ChangeDutyCycle(0)  # Stop the motor
                print("Object detected! Motor stopped.")
                motor_running = False  # Prevent restarting the motor
        
        time.sleep(0.2)  # Check distance every 200ms

except KeyboardInterrupt:
    print("\nStopping program.")
finally:
    left_pwm.ChangeDutyCycle(0)  # Stop the motor
    left_pwm.stop()
    GPIO.cleanup()
    print("Cleaned up GPIO and exited.")
