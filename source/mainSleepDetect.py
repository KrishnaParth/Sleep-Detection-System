#------------------------------------------------------
#File Name ---mainSleepDetect.py
#  This file contains functions to detect drowsiness using eye threshold and yawning
#

# by Seema Saini
#------------------------------------------------------


#Library functions to be imported

import scipy
from scipy.spatial import distance as dist
from imutils.video import VideoStream
from imutils import face_utils
import threading 
import numpy as np
import argparse
import imutils
import time
import dlib
import cv2
import os
import atexit
import queue

from mqttConnect import *
from buzzer import *
from motor import *
from alcoholDetection import *
from MQTTlog import *
from config import *
from headPosition import getHeadTiltAndCoords
from GPS import *

from RPLCD import CharLCD


#--------------------------------------------------------------------------------
    

def exit_handler():
    print("Cleaning Up...")
    
    if LCD:
        LCD_clear()
        LCD_close()
        
    if CLOUD_CONN:
        mqttt_disconnect(myMQTTClient)
    GPIO.cleanup()
    cv2.destroyAllWindows()
    vs.stop() 
    exit(1)


#This function helps to find midpoint between two points
def find_mid(l, r):
    return int((l[0] + r[0])/2), int((l[1] + r[1])/2)
    

#This function helps to find head Roll Angle 
def get_head_tilt_angle(shape, frame):
    #global frame
    (lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
    (rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

    leftEye = shape[lStart:lEnd]
    rightEye = shape[rStart:rEnd]

    leftEyeHull = cv2.convexHull(leftEye)
    rightEyeHull = cv2.convexHull(rightEye)
    #with frame_lock:
    cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
    cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
    left = find_mid(leftEye[0], leftEye[3])
    right = find_mid(rightEye[0], rightEye[3])
    hor_line1 = cv2.line(frame, left, right, (0, 0, 255), 2)
    #cv2.imshow("head Roll", frame) 

    # hor_line2 = cv2.line(frame, rightEye[0], rightEye[3], (0, 0, 255), 2)

    delta_x = right[0] - left[0]
    delta_y = right[1] - left[1]

    # Slope of line formula
    angle = np.arctan(delta_y / delta_x)

    # Converting radians to degrees
    angle = (angle * 180) / np.pi
    # print("left", leftEye)
    # print("right", rightEye)
    return angle
    
    
#This function is used to find euclidean distance between 2 sets of vertical eye coordinates
def eye_aspect_ratio(eye):
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])

    C = dist.euclidean(eye[0], eye[3])

    ear = (A + B) / (2.0 * C)

    return ear


#This function is used to average EAR for both eyes
def final_ear(shape):
    #Array Slices Indexes in order to extract the eye regions from the set of facial landmarks
    (lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]  
    (rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

    leftEye = shape[lStart:lEnd]
    rightEye = shape[rStart:rEnd]

    leftEAR = eye_aspect_ratio(leftEye)
    rightEAR = eye_aspect_ratio(rightEye)

    ear = (leftEAR + rightEAR) / 2.0
    return (ear, leftEye, rightEye)


def lip_distance(shape):
    #top_lip = shape[50:54]
    top_lip = shape[50:53]
    top_lip = np.concatenate((top_lip, shape[61:64]))

    #low_lip = shape[56:60]
    low_lip = shape[56:59]
    low_lip = np.concatenate((low_lip, shape[65:68]))

    top_mean = np.mean(top_lip, axis=0)
    low_mean = np.mean(low_lip, axis=0)

    distance = abs(top_mean[1] - low_mean[1])
    return distance


ap = argparse.ArgumentParser()  # required to parse argument if any
ap.add_argument('--webcam', type=int, default=0,help='Index of the Webcam Source (default: 0)')
args = vars(ap.parse_args())


image_points = np.array([
    (359, 391),     # Nose tip 34
    (399, 561),     # Chin 9
    (337, 297),     # Left eye left corner 37
    (513, 301),     # Right eye right corne 46
    (345, 465),     # Left Mouth corner 49
    (453, 469)      # Right mouth corner 55
], dtype="double")
    

#MQTT Connection Threads

myMQTTClient = None
mqtt_connected = False

global_var_lock = threading.Lock()


def mqtt_thread():
    global myMQTTClient, mqtt_connected
    if myMQTTClient is None:  
        myMQTTClient = mqtt_setup()  
        print("MQTTClient :: ", myMQTTClient)
        if myMQTTClient is not None:
            mqtt_connected = True
        else:
            mqtt_connected = False 


def is_internet_available():
    try:
        response = requests.get("http://www.google.com", timeout=5)
        return True if response.status_code == 200 else False
    except requests.RequestException:
        return False
        
        
def mqtt_ping_thread():
    global myMQTTClient, mqtt_connected
    while True:
        try:
            if is_internet_available():
                with global_var_lock:
                    if myMQTTClient is not None:
                        mqtt_connected = True
                        print("MQTT Ping Success: ", mqtt_connected)
                    else:
                        mqtt_thread() 
            else:
                with global_var_lock:
                    myMQTTClient = None
                    mqtt_connected = False
        except Exception as e:
            print("MQTT Ping Error: ", e)
            with global_var_lock:
                mqtt_connected = False
        time.sleep(10)

# Function to periodically call mqtt_ping_thread
def call_mqtt_ping_thread():
    while True:
        mqtt_ping_thread()
        time.sleep(20)  # Adjust the interval as needed            

# Create a Timer to periodically call mqtt_ping_thread
ping_timer = threading.Timer(0, call_mqtt_ping_thread)
ping_timer.daemon = True
ping_timer.start()
            

# Define global variables
mqtt_connected = False

# Define locks for synchronization
frame_lock = threading.Lock()
mqtt_connected_lock = threading.Lock()
gps_lock = threading.Lock()
file_lock = threading.Lock()
mqtt_lock = threading.Lock()
logs_lock = threading.Lock()

#Function to start Log Thread with different Arguments
def start_log_thread(Lat, Long, drowsy, sleepy, using_phone, drunk):
    with logs_lock:
        logs_thread = threading.Thread(target=SaveLogs, args=(Lat, Long, drowsy, sleepy, using_phone, drunk))
        logs_thread.daemon = True
        logs_thread.start()
        
        
#Function to start MQTT Publish Thread with different Arguments
def start_mqtt_pub_thread(myMQTTClient, Lat, Long, drowsy, sleepy, using_phone, drunk):
    mqtt_pub_t = threading.Thread(target=publish_data, args=(myMQTTClient, Lat, Long, drowsy, sleepy, using_phone, drunk))
    mqtt_pub_t.daemon = True
    mqtt_pub_t.start()
    
    
#Function to start MQTT Publish Thread with different Arguments
def start_motor_thread(motor, drowsy, sleepy):
    motor_t = threading.Thread(target=Motor_on, args=(motor, drowsy, sleepy))
    motor_t.daemon = True
    motor_t.start()

# Function for video streaming thread
def video_stream_thread(frame_queue, vs):
    while True:
        new_frame = vs.read()          
        
        if new_frame is None:
            continue

        frame = imutils.resize(new_frame, width=450)
        frame_queue.put(frame)
        
    vs.release()    

speed = None
motor = None
            

def main():
    global speed, motor
    
    alarm_status = False
    close_eye_count = 0
    yawn_status = False
    yawn_count = 0
    prev_frame_time = 0
    new_frame_time = 0
    frame_height = 450
    EAR_VAR_CHECK = 0
    
    buzzer_config(BUZZER_PIN,LED_PIN) #Set buzzer & LED - pin 23 & 25 as output
    motor = Motor_config(IN1, IN2, EN)
    alcohol_sensor_config(MQ3_PIN)
    
    if GPS:
        setup_GPS()
    
    atexit.register(exit_handler)
    
    LCD_clear()
    LCD_display("LOADING SETTINGS !!")
    time.sleep(1)
    print(" < Loading the detector > ")
    cloudTimer=0
    
    if DLIB:
        detector = dlib.get_frontal_face_detector()
    else:
        detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    
        
    LCD_clear()
    print(" < Loading the predictor >")
    time.sleep(1)
    
    predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')
    
    LCD_clear()
    print(" < Starting Real Time Video Streaming > ")
    
    # Initialize the video stream
    if WEBCAM:
        vs = VideoStream(src=args["webcam"]).start()  
    else:
        vs= VideoStream(usePiCamera=True).start()
        
    
    LCD_display("STARTING VIDEO  STREAMING!!")
    time.sleep(1)
            
    time_10s=time.time() 
    time_1min=time.time()
    time_5s=time.time()
        
    try:   
        while True:
            #print(f"Speed: {speed} km/hr")
            
            if CLOUD_CONN:
                if not is_file_empty() and mqtt_connected == True and myMQTTClient is not None:
                    with mqtt_lock:
                        # Start the File Handling thread
                        file_thread = threading.Thread(target=publish_payloads_from_file, args=(myMQTTClient,))
                        file_thread.daemon = True
                        file_thread.start()
                    #publish_payloads_from_file(myMQTTClient)
    
    
            LCD_clear()
            
            frame = vs.read()
            frame = imutils.resize(frame, width=450)
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
              
            size = gray.shape
    
            if DLIB:
                rects = detector(gray, 0)
            else:
                rects = detector.detectMultiScale(gray, scaleFactor=1.3, 
                    minNeighbors=1, minSize=(30, 30),
                    flags=cv2.CASCADE_SCALE_IMAGE)
          
            if ALCOHOL:
                if check_alcohol_detection() == True:
                    if CLOUD_CONN:
                        if mqtt_connected:
                            print ('Message Sent To Owner...')
                            if(((int(time.time()-cloudTimer))>PUBLISH_TIME) or (cloudTimer==0)):
                                start_mqtt_pub_thread(myMQTTClient,Lat, Long, False, False, False, True)
                                #publish_data(myMQTTClient,Lat, Long, False, False, False, True)
                                cloudTimer=time.time()
                        else:
                            start_log_thread(Lat, Long, False, False, False, True)
                            #SaveLogs(Lat, Long, False, False, False, True)
                
    
        ###for eye and yawn detection of all the faces detected in the frame
            if(len(rects)>0):
                if DLIB:  
                    x, y, h, w = rects[0].left(), rects[0].top(), rects[0].right(), rects[0].bottom()
                    cv2.rectangle(frame, (x, y), (h, w), (255, 255, 255), 1)
                    rect = dlib.rectangle(int(x), int(y), int(h),int(w))    
                else:     
                    # for eye and yawn detection of only prominent face
                    (x, y, w, h) = rects[0]
                    rect = dlib.rectangle(int(x), int(y), int(x + w),int(y + h))    
    
                if GPS:
                    gps_data, speed = getLatLong()
                    
                    if gps_data is not None and gps_data:
                        lat = gps_data.get('lat')
                        lon = gps_data.get('long')
                        if(lat != 0 and lon !=  0 and lat != Lat & lon != Long):
                            modify_gps_global(gps_data)
                            print("GPS : ", gps_data['lat'], gps_data['long'])
                
                shape = predictor(gray, rect)
                shape = face_utils.shape_to_np(shape)
    
                eye = final_ear(shape)
                ear = eye[0]
                leftEye = eye [1]
                rightEye = eye[2]
                
    
                distance = lip_distance(shape)
    
                leftEyeHull = cv2.convexHull(leftEye)
                rightEyeHull = cv2.convexHull(rightEye)
                cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
                cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
    
                lip = shape[48:60]
                cv2.drawContours(frame, [lip], -1, (0, 255, 0), 1)
             
             
                #To Find if person is using phone
                
                headTiltAngle = get_head_tilt_angle(shape, frame)
    
                if headTiltAngle > 20 and int(time.time()-time_5s) > 3:
                    cv2.putText(frame, 'LEFT TILT : USING PHONE' + str(int(headTiltAngle)) + ' degrees',
                               (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                               (0, 0, 0), 2, cv2.LINE_4)
                    
                    start_motor_thread(motor, True, False)
                    buzzer_on(False,False,True)
                    
                    #Motor_on(motor, True, False)
                        
                    if CLOUD_CONN:
                        if mqtt_connected:
                            print ('Message Sent To Owner...')
                            if(((int(time.time()-cloudTimer))>PUBLISH_TIME) or (cloudTimer==0)):
                                start_mqtt_pub_thread(myMQTTClient,Lat, Long, False, False, True, False)
                                #publish_data(myMQTTClient,Lat, Long, False, False, True, False)
                                cloudTimer=time.time()
                        else:
                            start_log_thread(Lat, Long, False, False, True, False)
                            #SaveLogs(Lat, Long, False, False, True, False)
                            
                elif headTiltAngle < -20 and int(time.time()-time_5s) > 3:
                    cv2.putText(frame, 'RIGHT TILT : USING PHONE' + str(int(headTiltAngle)) + ' degrees',
                               (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                               (0, 0, 0), 2, cv2.LINE_4)
                    
                    start_motor_thread(motor, True, False)
                    buzzer_on(False,False,True)
                    
                    #Motor_on(motor, True, False)
                        
                    if CLOUD_CONN:
                        if mqtt_connected:
                            print ('Message Sent To Owner...')
                            if(((int(time.time()-cloudTimer))>PUBLISH_TIME) or (cloudTimer==0)):
                                start_mqtt_pub_thread(myMQTTClient,Lat, Long, False, False, True, False)
                                #publish_data(myMQTTClient,Lat, Long, False, False, True, False)
                                cloudTimer=time.time()
                        else:
                            start_log_thread(Lat, Long, False, False, True, False)
                            #SaveLogs(Lat, Long, False, False, True, False)
                        
                            
                elif -10 < headTiltAngle < 10:
                    time_5s = time.time()
                    buzzer_on(False,False,False)
                    #Motor_off(motor)
                    
                    '''
                    cv2.putText(frame, 'STRAIGHT :' + str(int(headTiltAngle)) + ' degrees', (20, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1,
                               (0, 0, 0), 2, cv2.LINE_4)
                    '''
                
                # Closed Eye Detection
                
                if ((ear < EYE_AR_THRESH) and (yawn_status==False)):
                    close_eye_count += 1
                    if close_eye_count >= EYE_AR_CONSEC_FRAMES:
                        cv2.putText(frame, "DROWSINESS ALERT : Closed Eyes!", (10, 30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        
                        if alarm_status == False:
                            alarm_status = True
                            print("Drowsiness Alert:Close Eye Detected")
                            
                            #if LCD:
                                #LCD_display("Awake Awake !!")
                            
                            start_motor_thread(motor, True, False)
                            buzzer_on(True, False, False)
                            
                            #Motor_on(motor, True, False)
                            
                            if CLOUD_CONN:
                                if mqtt_connected:
                                    print ('Message Sent To Owner...')
                                    if(((int(time.time()-cloudTimer))>PUBLISH_TIME) or (cloudTimer==0)):
                                        start_mqtt_pub_thread(myMQTTClient,Lat, Long, False, True, False, False)
                                        #publish_data(myMQTTClient,Lat, Long, False, True, False, False)
                                        cloudTimer=time.time()
                                else :
                                    start_log_thread(Lat, Long, False, True, False, False)
                                    #SaveLogs(Lat, Long, False, True, False, False)
                                    
                            time_10s = time.time()
                                    
                else:
                    EAR_VAR_CHECK+=1
                    if(EAR_VAR_CHECK >=2):
                        close_eye_count = 0
                        EAR_VAR_CHECK=0
                    alarm_status = False
                    buzzer_on(False,False, False)
                    #Motor_off(motor)
                    
                    #if LCD:
                        #LCD_clear();
                    
    
                if (distance > YAWN_THRESH):
                    cv2.putText(frame, "DROWSINESS ALERT : Yawning ", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
                    if yawn_count==0:
                        time_1min=time.time()        
                                        
                    if yawn_status == False :
                        yawn_status = True
                        yawn_count+=1
                        print("Drowsiness Alert : Yawning!",yawn_count,int(time.time()-time_1min))
                        
                        #Another Yawn is detected within time window YAWN_ALERT_TIME [30s]
                        if(((int(time.time()-time_1min))<=YAWN_ALERT_TIME) and yawn_count>=YAWN_ALERT_COUNT):
                            
                            #if LCD:
                                #LCD_display("Yawning! Awake !!")
                            
                            start_motor_thread(motor, True, False)
                            buzzer_on(True,False, False)
                            
                            #Motor_on(motor, True, False)
                            
                            if CLOUD_CONN:
                                if mqtt_connected:
                                    print ('Message Sent To Owner...')
                                    if(((int(time.time()-cloudTimer))>PUBLISH_TIME) or (cloudTimer==0)):
                                        start_mqtt_pub_thread(myMQTTClient,Lat, Long, True, False, False, False)
                                        #publish_data(myMQTTClient,Lat, Long, False, True, False, False)
                                        cloudTimer=time.time()
                                else:
                                    start_log_thread(Lat, Long, True, False, False, False)
                                    #SaveLogs(Lat, Long, True, False, False, False)
                                    
                            time_10s = time.time()
                                
                            yawn_count=0
                                    
                        elif(((int(time.time()-time_1min))>YAWN_ALERT_TIME) and yawn_count<YAWN_ALERT_COUNT):
                            yawn_count=0
                            
                        elif(((int(time.time()-time_1min))>YAWN_ALERT_TIME) and yawn_count>YAWN_ALERT_COUNT):
                            yawn_count=0
                 
                else:
                    yawn_status=False
                    buzzer_on(False,False, False)
                    #Motor_off(motor)
                    
                    #if LCD:
                        #LCD_clear();
                    
                for (i, (x, y)) in enumerate(shape):
                    if i == 33:
                        # something to our key landmarks
                        # save to our new key point list
                        # i.e. keypoints = [(i,(x,y))]
                        image_points[0] = np.array([x, y], dtype='double')
                    elif i == 8: #Nose tip
                        # something to our key landmarks
                        # save to our new key point list
                        # i.e. keypoints = [(i,(x,y))]
                        image_points[1] = np.array([x, y], dtype='double')
                    elif i == 36:
                        # something to our key landmarks
                        # save to our new key point list
                        # i.e. keypoints = [(i,(x,y))]
                        image_points[2] = np.array([x, y], dtype='double')          
                    elif i == 45:
                        # something to our key landmarks
                        # save to our new key point list
                        # i.e. keypoints = [(i,(x,y))]
                        image_points[3] = np.array([x, y], dtype='double')
                    elif i == 48:
                        # something to our key landmarks
                        # save to our new key point list
                        # i.e. keypoints = [(i,(x,y))]
                        image_points[4] = np.array([x, y], dtype='double')
                    elif i == 54:
                        # something to our key landmarks
                        # save to our new key point list
                        # i.e. keypoints = [(i,(x,y))]
                        image_points[5] = np.array([x, y], dtype='double')
    
                (head_tilt_degree, start_point, end_point,end_point_alt) = getHeadTiltAndCoords(size, image_points, frame_height)
                
                print("{CloseEyeCount}, {YawnStatus}, {HeadTilt}".format(CloseEyeCount=close_eye_count,YawnStatus=yawn_status, HeadTilt=head_tilt_degree[0:4]))
    
                if((head_tilt_degree>HEAD_TILT_DOWNANGLE) and (head_tilt_degree<HEAD_TILT_UPANGLE) and (close_eye_count>HEAD_EYE_CLOSE_COUNT)):
                    cv2.putText(frame, "SLEEPINESS : Head Down & Closed eyes!", (10, 30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    print("Sleepiness Alert:Head Down & Eye Clousure Detected")            
                    
                    #if LCD:
                        #LCD_display("Awake Awake !!")
                    
                    start_motor_thread(motor, False, True)
                    buzzer_on(False,True, False)
                    
                    #Motor_on(motor, False, True)
                    cv2.putText(frame, "SLEEPINESS : Head Down & Closed eyes!", (10, 30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    if CLOUD_CONN:
                        if mqtt_connected:
                            if(((int(time.time()-cloudTimer))>PUBLISH_TIME)or (cloudTimer==0)):
                                print ('Message Sent To Owner...')
                                start_mqtt_pub_thread(myMQTTClient,Lat, Long, False, True, False, False)
                                #publish_data(myMQTTClient,Lat, Long, False, True, False, False)
                                cloudTimer=time.time()
                        else:
                            start_log_thread(Lat, Long, False, True, False, False)
                            #SaveLogs(Lat, Long, False, True, False, False)
                    
                    time_10s = time.time()
                
                #if LCD:
                    #LCD_clear();
                        
                new_frame_time = time.time()
                fps = 1/(new_frame_time-prev_frame_time)
                prev_frame_time = new_frame_time
                fps = int(fps)
                text = " FPS: " + str(fps)
                #print(text)
                
                cv2.putText(frame, "EAR: {:.2f}".format(ear), (0, 270),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2,cv2.LINE_AA,0)
                cv2.putText(frame, "YAWN: {:.2f}".format(distance), (0, 300),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2,cv2.LINE_AA,0)
                cv2.putText(frame, text, (0, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2,cv2.LINE_AA,0)
            
                
                winname = "Frame"
                cv2.namedWindow(winname)        
                cv2.moveWindow(winname, 40,30)
                if frame is not None: 
                    cv2.imshow(winname, frame) 
                    key = cv2.waitKey(1) & 0xFF
                
                
                if CLOUD_CONN:
                    if(time.time()-time_10s>=90): 
                        LCD_clear()
    
                        if mqtt_connected:
                            start_mqtt_pub_thread(myMQTTClient,Lat, Long, False, False, False, False)
                            #publish_data(myMQTTClient,Lat, Long, False, False, False, False)
                            print ('Message Sent To Owner : Active...')
                        else:
                            start_log_thread(Lat, Long, False, False, False, False)
                            #SaveLogs(Lat, Long, False, False, False, False)
    
                        time_10s = time.time()
                        LCD_display("ACTIVE !!")
                        time.sleep(1)
                        LCD_clear()
            
                if key == ord("q"):
                    if GPS:
                        close_gps()
                    
                    if MOTOR:
                        Motor_off(motor)
                        motor.stop()
            
                    if LCD:
                        LCD_clear()
                        LCD_close()
                    
                    if CLOUD_CONN:
                        mqttt_disconnect(myMQTTClient)
                    
                    GPIO.cleanup()
                    cv2.destroyAllWindows()
                    vs.stop() 
                    break
    
    except KeyboardInterrupt:
        if GPS:
            close_gps()
        
        if CLOUD_CONN:
            mqtt_disconnect(myMQTTClient)

        if LCD:
            LCD_clear()
            LCD_close()
            
        if MOTOR:
            Motor_off(motor)
            motor.stop()

        cv2.destroyAllWindows()
        GPIO.cleanup()
        vs.stop()    

        pass


  
if __name__ == "__main__":
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    
    '''
    # Start the video stream thread
    frame_queue = queue.Queue()   
    
    video_thread = threading.Thread(target=video_stream_thread, args=(frame_queue, vs))
    video_thread.daemon = True
    video_thread.start()
    
    main(frame_queue, vs)
    
    #video_thread.join()
    '''
    main()
    
    if GPS:
        close_gps()
        
    if CLOUD_CONN:
        mqtt_disconnect(myMQTTClient)

    if LCD:
        LCD_clear()
        LCD_close()
        
    if MOTOR:
        motor.stop()

    cv2.destroyAllWindows()
    GPIO.cleanup()
    vs.stop()    


