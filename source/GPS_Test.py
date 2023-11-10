import serial
import time 
import string
import pynmea2  
import re
import datetime
import math

def updateconfig(config_file_path, lat_long_dict):
    # Read the contents of the config.py file
    with open(config_file_path, 'r') as file:
        config_content = file.read()

    # Use regular expressions to find and update Latitude and Longitude values
    latitude_pattern = r'Lat\s*=\s*([\d.-]+)'
    longitude_pattern = r'Long\s*=\s*([\d.-]+)'

    # Find current Latitude and Longitude values using regular expressions
    current_latitude_match = re.search(latitude_pattern, config_content)
    current_longitude_match = re.search(longitude_pattern, config_content)

    if current_latitude_match and current_longitude_match:
        # Extract current Latitude and Longitude values from matches
        current_latitude = float(current_latitude_match.group(1))
        current_longitude = float(current_longitude_match.group(1))
    else:
        print("Error: 'LATITUDE' or 'LONGITUDE' pattern not found in config.py.")
        return

    # Update Latitude and Longitude with values from lat_long_dict
    new_latitude = lat_long_dict.get('lat', current_latitude)
    new_longitude = lat_long_dict.get('long', current_longitude)

    # Replace the old Latitude and Longitude values with the new ones in the config_content
    config_content = re.sub(latitude_pattern, f'Lat = {new_latitude}', config_content)
    config_content = re.sub(longitude_pattern, f'Long = {new_longitude}', config_content)

    # Write the updated content back to the config.py file
    with open(config_file_path, 'w') as file:
        file.write(config_content)        


def haversine_distance(lat1, lon1, lat2, lon2):
    # Radius of the Earth in meters
    R = 6371000

    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Calculate differences in latitude and longitude
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Calculate haversine of half the angular distances
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Calculate the distance
    distance = R * c
    return distance

def calculate_speed(lat1, lon1, lat2, lon2, time_diff):
    distance = haversine_distance(lat1, lon1, lat2, lon2)
    speed = distance / time_diff
    return speed / 3.6


prev_lat, prev_lon, prev_time = None, None, None
moving_threshold = 5
timestamp = None
        
while True:
    try:
        prev_time = datetime.datetime.now()
        ser=serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)
        dataout =pynmea2.NMEAStreamReader() 
        newdata=ser.readline()
        lat_long_dict={}
        #print(newdata)
        if '$GPRMC' in str(newdata):
            #print("utf8 : ",newdata)
            try:
                newmsg=pynmea2.parse(newdata.decode('utf-8')) 
            except  (UnicodeDecodeError, pynmea2.ParseError) as e:
                print("Error Decoding Data: ",e)
                newmsg = None
    
            if newmsg is not None:
                lat=newmsg.latitude 
                lng=newmsg.longitude
            else:
                print("newmsg = None")
                lat=0
                lng=0
                    
            gps = "Latitude=" + str(lat) + "and Longitude=" +str(lng) 
            print('LAT :', lat, ' LON :', lng)
            print(gps)
            lat_long_dict['lat']= float(lat)
            lat_long_dict['long']= float(lng)
            
        lat = 37.7749
        lng = -122.4194
        prev_lat = 34.0522
        prev_lon = -118.2437
    
        if prev_lat is not None and prev_lon is not None and prev_time is not None:
            timestamp = datetime.datetime.now()
            time_diff = (timestamp - prev_time).total_seconds()
            speed = calculate_speed(prev_lat, prev_lon, lat, lng, time_diff)
            print(f"Speed: {speed:.2f} km/hr")
            
            if speed > moving_threshold:
                print("Car is moving")
            else:
                print("Car is stationary")

        
        #prev_lat, prev_lon, prev_time = lat, lng, timestamp
                
    except serial.SerialException as se:
        print("Serial Port Error:", se)
    finally:
        ser.close()

        #updateconfig("/home/rpi/Desktop/De-Sleep/test.py", lat_long_dict)


