import math
import json
import datetime
#from config import *
from mqttConnect import *
import os


__all__ = ['is_file_empty', 'SaveLogs', 'publish_payloads_from_file']


# Specify the file path
file_path = "./MQTTLogs.txt"

def is_file_empty():
    return os.path.getsize(file_path) == 0

def SaveLogs(Lat, Long, drowsy, sleepy, using_phone, drunk):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(file_path, 'a') as file:
        if drowsy:
            message = {"timestamp": timestamp,"Status" : "Drowsy", "lat" : Lat, "long" : Long}
            
        elif sleepy:
            message = {"timestamp": timestamp,"Status" : "Sleepy", "lat" : Lat, "long" : Long}
            
        elif using_phone:
            message = {"timestamp": timestamp,"Status" : "Using Phone", "lat" : Lat, "long" : Long}
        
        elif drunk:
            message = {"timestamp": timestamp,"Status" : "User Drunk", "lat" : Lat, "long" : Long}
            
        else:
            message = {"timestamp": timestamp,"Status" : "Active", "lat" : Lat, "long" : Long}
    
        message_str = json.dumps(message, indent=4)  
        file.write(message_str + '\n')
        #file.write(message) 

    print("Message has been stored in the file:", file_path)


def publish_payloads_from_file(myMQTTClient):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        buffer = []
        for line in lines:
            line = line.strip()
            if line == '}':
                buffer.append(line)
                if buffer:
                    try:
                        payload = json.loads('\n'.join(buffer))
                        payload_str = json.dumps(payload)
                        publish_payload(myMQTTClient, payload_str)
                        #with open("./logfile.txt", 'a') as file:
                            #message_str = json.dumps(message, indent=4)  
                            #file.write(str(payload) + '\n')
                        # print(payload)
                    except json.JSONDecodeError:
                        print("Error decoding JSON:", buffer)
                buffer = []
            else:
                buffer.append(line)

    # Rewrite the file with the remaining lines (lines that were not published)
    with open(file_path, 'w') as file:
        pass # file.writelines(lines[len(buffer):])

# def main():
#     Lat =  # Replace with actual latitude
#     Long = # Replace with actual longitude
#     SaveLogs(Lat, Long, True, False, False, False)
#     publish_payloads_from_file(True)

# if __name__ == "__main__":
#     main()
       
    
