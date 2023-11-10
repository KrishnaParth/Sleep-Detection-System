import serial
import string
import pynmea2
import re
import math
import datetime


#GPS Global Param, default : RSYS Noida
Lat = 28.60970044
Long = 77.36303337

prev_lat, prev_lon, prev_time = None, None, None
moving_threshold = 18
timestamp = None
vehicle_speed = None
ser = None

def setup_GPS():
	global ser
	try:
		ser=serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)
		dataout =pynmea2.NMEAStreamReader()
		#print(ser)
	except serial.SerialException as se:
		print("Serial Port Error:", se)
	#finally:
		#ser.close()

def close_gps():
	global ser
	ser.close()
	
def modify_gps_global(gps_data):
    global Lat, Long
    lat = gps_data.get('lat')
    lon = gps_data.get('long')

    if lat is not None and lon is not None:
        Lat = lat
        Long = lon
        #print("Latitude:", lat)
        #print("Longitude:", lon)
    else:
        print("Invalid GPS data. Latitude or longitude missing.")
        

#This function sets up GPS, returns Lattitude & Longitude

def getLatLong():
    global ser, vehicle_speed, prev_lat, prev_lon, prev_time
    lat_long_dict = {}
    
    if ser is not None:
        newdata = ser.readline()
        if '$GPRMC' in str(newdata):
            try:
                newmsg = pynmea2.parse(newdata.decode('utf-8')) 
            except (UnicodeDecodeError, pynmea2.ParseError) as e:
                print("Error Decoding Data:", e)
                newmsg = None

            if newmsg is not None:
                lat = newmsg.latitude 
                lng = newmsg.longitude
                gps = f"Latitude={lat} and Longitude={lng}" 
                lat_long_dict['lat'] = float(lat)
                lat_long_dict['long'] = float(lng)

                if prev_lat is not None and prev_lon is not None and prev_time is not None:
                    timestamp = datetime.datetime.now()
                    time_diff = (timestamp - prev_time).total_seconds()
                    vehicle_speed = calculate_speed(prev_lat, prev_lon, lat, lng, time_diff) * 3.6
                    #print(f"Speed: {vehicle_speed:.2f} km/hr")

                    if vehicle_speed > moving_threshold:
                        print("Car is moving")
                    else:
                        print("Car is stationary")
                    prev_lat, prev_lon, prev_time = lat, lng, timestamp
                else:
                    prev_lat, prev_lon, prev_time = lat, lng, datetime.datetime.now()
            else:
                lat_long_dict = None
        else:
            lat_long_dict = None
    else:
        print("ser is None")
        lat_long_dict = None
    
    return lat_long_dict, vehicle_speed



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
    return speed

'''
# Example usage
lat1 = 37.7749
lon1 = -122.4194
lat2 = 34.0522
lon2 = -118.2437
time_diff = 3600  # 1 hour in seconds
speed = calculate_speed(lat1, lon1, lat2, lon2, time_diff)
print(f"Speed: {speed:.2f} m/s")
'''
