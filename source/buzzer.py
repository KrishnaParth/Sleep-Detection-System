

#------------------------------------------------------
#File Name ---buzzer.py
#  This file contains functions to play buzzer on GPIO
#

#------------------------------------------------------
#Libraries
import RPi.GPIO as GPIO
from time import sleep
from config import *
from RPLCD import CharLCD


buzzer=16  #23 BCM
led=22     #25 BCM

if LCD:
    #lcd = CharLCD(cols=16, rows=2, pin_rs=37, pin_e=35, pins_data=[40, 38, 36, 32, 33, 31, 29, 23])
    GPIO.setmode(GPIO.BOARD)
    lcd = CharLCD(cols=16, rows=2, pin_rs=37, pin_e=35, pins_data=[33, 31, 29, 23], numbering_mode=GPIO.BOARD)

def LCD_display(message):  
    lcd.clear()
    #lcd.cursor_pos = (0, 0)  
    lcd.write_string(message)
    '''
    for i in range(5):
        sleep(0.5)
        lcd.clear()
        sleep(0.5)
        i +=1
    '''

def LCD_clear():
    lcd.clear()
    
def LCD_close():
    lcd.close(True)
    
    
is_sound_on = False
def buzzer_config(x,y):
    #Disable warnings (optional)
    GPIO.setwarnings(False)
    #Select GPIO mode
    #GPIO.setmode(GPIO.BCM)
    GPIO.setmode(GPIO.BOARD)
    buzzer=x
    led=y
    GPIO.setup(buzzer,GPIO.OUT)
    GPIO.setup(led,GPIO.OUT)
    
def buzzer_start():
    GPIO.output(led,GPIO.HIGH)
    GPIO.output(buzzer,GPIO.HIGH)
    print ("Beep")
    sleep(2) # Delay in seconds
    GPIO.output(buzzer,GPIO.LOW)
    sleep(0.5)

def buzzer_Off():
    is_sound_on = False
    LCD_clear()
    GPIO.output(led,GPIO.LOW)
    GPIO.output(buzzer,GPIO.LOW)

def toggle_sound():
    global is_sound_on
    is_sound_on = not is_sound_on
    GPIO.output(buzzer,is_sound_on)
    GPIO.output(led,is_sound_on)
    
def buzzer_on(drowsiness, sleepiness, usingPhone):
    LCD_clear()
    if drowsiness:
        try:
            for i in range(5):
                LCD_display("DROWSINESS ALERT!!")
                toggle_sound()
                sleep(1)
                LCD_clear()
                i += 1
        except KeyboardInterrupt:
            buzzer_Off()
            GPIO.cleanup()
            
    elif sleepiness:
        #buzzer_start()
        try:
            for i in range(50):
                LCD_display("SLEEPINESS ALERT!!")
                toggle_sound()
                sleep(0.1)
                LCD_clear()
                i += 1
        except KeyboardInterrupt:
            buzzer_Off()
            GPIO.cleanup()
    
    if usingPhone:
        try:
            for i in range(5):
                LCD_display("USING PHONE!!")
                toggle_sound()
                sleep(1)
                LCD_clear()
                i += 1
        except KeyboardInterrupt:
            buzzer_Off()
            GPIO.cleanup()
        
    else:
        buzzer_Off()
        
        
