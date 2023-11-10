#------------------------------------------------------
#File Name ---mqttConnect.py
#  This file contains functions to send data to cloud
#

#------------------------------------------------------

import math
import requests
import json
import datetime
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from config import *
import threading
import os
import time
import socket

#-------------------------------------------------------------

#MQTT_TOPIC="sdk/test/python" # POC/De-Sleep
#MQTT_ENDPOINT="a37j1ob20jgo2h-ats.iot.eu-north-1.amazonaws.com"

MQTT_TOPIC="desleep/cams/krishnacam" # POC/De-Sleep
MQTT_ENDPOINT="a37j1ob20jgo2h-ats.iot.us-east-1.amazonaws.com"

def mqtt_setup():
    print(" < Setting up MQTT Connection > ")
    #global mqtt_connected, myMQTTClient
    myMQTTClient = AWSIoTMQTTClient("basicPubSub")  #basicPubSub DeSleep
    myMQTTClient.configureEndpoint(MQTT_ENDPOINT, 8883)

    myMQTTClient.configureCredentials("/home/rpi/Desktop/De-Sleep/certificates/jsnode/root-CA.crt", "/home/rpi/Desktop/De-Sleep/certificates/jsnode/krishnacam.private.key", "/home/rpi/Desktop/De-Sleep/certificates/jsnode/krishnacam.cert.pem")
    #myMQTTClient.configureCredentials("/home/rasp-14841/Desktop/certificates/py/root-CA.crt", "/home/rasp-14841/Desktop/certificates/py/RaspberryPi.private.key", "/home/rasp-14841/Desktop/certificates/py/RaspberryPi.cert.pem")
    
    myMQTTClient.configureOfflinePublishQueueing(-1) # Infinite offline Publish queueing
    myMQTTClient.configureDrainingFrequency(2) # Draining: 2 Hz
    myMQTTClient.configureConnectDisconnectTimeout(10) # 10 sec
    myMQTTClient.configureMQTTOperationTimeout(5) # 5 sec
    print ('Trying Data Transfer From Raspberry Pi...')
    try:
        myMQTTClient.connect()       
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        #print(timestamp);
        #payload = json.dumps({"timestamp": timestamp})
        message = {"timestamp": timestamp,"Status" : "Active"} #"Message" : "User is feeling Active"
        myMQTTClient.publish(topic=MQTT_TOPIC, payload=json.dumps(message), QoS=0)
        
    except Exception as e:
        # with global_var_lock:
            # mqtt_connected = False
        print("MQTT Connection Error: {}".format(str(e)))
        myMQTTClient = None

    return myMQTTClient
    

def is_internet_available():
    try:
        # Attempt to create a socket to a known server (e.g., Google's DNS server)
        socket.create_connection(("8.8.8.8", 53), timeout=5)
        #print("Socket: true", mqtt_connected)
        return True
    except OSError:
        #print("Socket: false", mqtt_connected)
        return False
            
def publish_data(myMQTTClient,Lat, Long, drowsy, sleepy, using_phone, drunk):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    #print("MQTT", Lat, Long)
        
    if drowsy:
        message = {"timestamp": timestamp,"Status" : "Drowsy", "lat" : Lat, "long" : Long}  #,"Message" : "User is feeling Drowsy"
        myMQTTClient.publish(topic=MQTT_TOPIC, payload=json.dumps(message), QoS=0)
        
    elif sleepy:
        message = {"timestamp": timestamp,"Status" : "Sleepy", "lat" : Lat, "long" : Long}  #,"Message" : "User is feeling sleepy"
        myMQTTClient.publish(topic=MQTT_TOPIC, payload=json.dumps(message), QoS=0)
        
    elif using_phone:
        message = {"timestamp": timestamp,"Status" : "Using Phone", "lat" : Lat, "long" : Long}  #,"Message" : "User is feeling sleepy"
        myMQTTClient.publish(topic=MQTT_TOPIC, payload=json.dumps(message), QoS=0)
    
    elif drunk:
        message = {"timestamp": timestamp,"Status" : "User Drunk", "lat" : Lat, "long" : Long}  #,"Message" : "User is Drunk"
        myMQTTClient.publish(topic=MQTT_TOPIC, payload=json.dumps(message), QoS=0)
        
    else:
        message = {"timestamp": timestamp,"Status" : "Active", "lat" : Lat, "long" : Long}  #,"Message" : "User is feeling sleepy"
        myMQTTClient.publish(topic=MQTT_TOPIC, payload=json.dumps(message), QoS=0)
        


def publish_payload(myMQTTClient, data):
    myMQTTClient.publish(topic=MQTT_TOPIC, payload=data, QoS=0)
    print("MQTT Disconnected Payload Published ..!! ")


def mqtt_disconnect(myMQTTClient):
    myMQTTClient.disconnect()



    
