import time
import RPi.GPIO as GPIO

# Set up GPIO
GPIO.setmode(GPIO.BOARD)
MQ3_PIN = 11  # Use the physical pin number you're connecting to
GPIO.setup(MQ3_PIN, GPIO.IN)

# Warm-up time for the sensor
WARM_UP_TIME = 2

# Read interval
READ_INTERVAL = 2  # seconds

def alcohol_sensor_config(MQ3_PIN):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(MQ3_PIN, GPIO.IN)
    time.sleep(WARM_UP_TIME) 
    print("Alcohol Sensor Configured ...!!")

def check_alcohol_detection():
    alcohol_level = GPIO.input(MQ3_PIN)  # Read digital value

    if alcohol_level:
        # print("No Alcohol detected!")
        return False
        
    else:
        print("Alcohol detected.")
        return True
    
'''
# Test Sensor

def read_mq3():
    try:
        print("Warming up sensor...")
        time.sleep(WARM_UP_TIME)
        print("Sensor warmed up.")

        while True:
            alcohol_level = GPIO.input(MQ3_PIN)  # Read digital valu
            print(alcohol_level)
            
            
            
            if alcohol_level:
                print("No Alcohol detected!")
            else:
                print("Alcohol detected.")
            
            time.sleep(READ_INTERVAL)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        GPIO.cleanup()
        
        
if __name__ == "__main__":
    read_mq3()
'''
