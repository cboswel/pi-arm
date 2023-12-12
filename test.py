import RPi.GPIO as GPIO
import time

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set the GPIO pin for the servo
servo_pin = 18
GPIO.setup(servo_pin, GPIO.OUT)

# Create a PWM instance with a frequency of 50Hz
pwm = GPIO.PWM(servo_pin, 50)

# Function to set the angle of the servo motor
def set_angle(angle):
    duty_cycle = (angle / 18) + 2
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Give the servo some time to move

# Main program
try:
    pwm.start(0)  # Start PWM with 0% duty cycle (neutral position)
    
    while True:
        # Move the servo to 0 degrees
        set_angle(0)
        time.sleep(1)

        # Move the servo to 90 degrees
        set_angle(90)
        time.sleep(1)

        # Move the servo to 180 degrees
        set_angle(180)
        time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()  # Stop PWM
    GPIO.cleanup()  # Cleanup GPIO
