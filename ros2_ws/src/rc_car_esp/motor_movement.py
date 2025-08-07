import Jetson.GPIO as GPIO
import time

# Choose your Jetson pins (physical board pin numbers)
PWM_PIN1 = 33  # Example pin (BOARD pin number) for motor 1
PWM_PIN2 = 32  # Example pin for motor 2

# REV-11-1200 expects ~1.0–2.0 ms pulses at 50 Hz
FREQ = 50  # Hz

def pulse_ms_to_dutycycle(pulse_ms):
    # Duty cycle in percent for GPIO.PWM
    period_ms = 1000.0 / FREQ
    return (pulse_ms / period_ms) * 100.0

# Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PWM_PIN1, GPIO.OUT)
GPIO.setup(PWM_PIN2, GPIO.OUT)

pwm1 = GPIO.PWM(PWM_PIN1, FREQ)
pwm2 = GPIO.PWM(PWM_PIN2, FREQ)

pwm1.start(0)
pwm2.start(0)

try:
    # Example: 1.6 ms pulse = slow forward
    duty = pulse_ms_to_dutycycle(1.6)
    pwm1.ChangeDutyCycle(duty)
    pwm2.ChangeDutyCycle(duty)

    time.sleep(5)  # run for 5 seconds

finally:
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()