
#------------------------------------------------------
#File Name ---buzzer.py
#  This file contains configuration for drowsinessdetection system
#

#------------------------------------------------------

#Setting the configuration parameters for drowsiness detection


DLIB=False
SENSITIVE=True
CLOUD_CONN=True 
PUBLISH_TIME=10 #60
WEBCAM=True
MOTOR=True
ALCOHOL=True
LCD=True
GPS=True

# Using Motor Driver L298N
IN1 = 18 #24 BCM
IN2 = 15 #23 BCM
EN = 13 #25 BCM

MQ3_PIN = 11

BUZZER_PIN=16 #23 BCM
LED_PIN=22 #25 BCM

#parameters for setting threshold values for eye and yawn detection
if SENSITIVE:
    EYE_AR_THRESH =0.26       #Eyes Aspect Ratio Threshold
    EYE_AR_CONSEC_FRAMES = 8 #NO of Consecutiv Frames Eyes must be Closed to be considered drowsy 15
    YAWN_THRESH = 17  #17
else:
    EYE_AR_THRESH =0.22
    EYE_AR_CONSEC_FRAMES = 20
    YAWN_THRESH = 20

#parameters for alarm play on yawn detection
YAWN_ALERT_TIME=30 #30
YAWN_ALERT_COUNT=2 #4


#configuration for drowsiness detction based on head down & up & number of eye closure
HEAD_TILT_DOWNANGLE=22  #22
HEAD_TILT_UPANGLE = 200 #200
HEAD_EYE_CLOSE_COUNT=1 #2


