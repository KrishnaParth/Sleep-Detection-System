import RPi.GPIO as GPIO          
from time import sleep

# Using Motor Driver L298N
in1 = 18 #24
in2 = 15 #23
en = 13 #25
temp1=1

p=None

def Motor_config(x, y, z):
        global p
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(x,GPIO.OUT)
        GPIO.setup(y,GPIO.OUT)
        GPIO.setup(z,GPIO.OUT)
        GPIO.output(x,GPIO.LOW)
        GPIO.output(y,GPIO.LOW)
        p=GPIO.PWM(z,1000)
        p.start(25)
        print("Motor Configured ...!!")
        return p
        

#p.start(25)
#print("\n")
#print("The default speed & direction of motor is LOW & Forward.....")
#print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
#print("\n")    


def Motor_off(p):
        #p.stop()
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        #print("Motor OFF ...!!")
                        

def Motor_on(p, drowsy, Sleepy):
        #print("p", p)
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        
        try:
                if drowsy:
                        #print("Motor Drowsy ...!!")
                        p.ChangeDutyCycle(50)
                        sleep(5)
                elif Sleepy:
                        #print("Motor Sleepy ...!!")
                        p.ChangeDutyCycle(75)
                        sleep(5)
                        
        except KeyboardInterrupt:
                p.stop()
                GPIO.output(in1,GPIO.LOW)
                GPIO.output(in2,GPIO.LOW)
                Motor_off(p)
        Motor_off(p)
        

#Test Motor
  
if __name__ == "__main__":
        
        Motor_config(in1, in2, en)
        
        while(1):

            x=input("Enter Command: ")
            print("Command : ", x)
            
            if x=='x':
                print("drowsy")
                Motor_on(p, True, False)
                
            if x=='y':
                print("sleepy")
                Motor_on(p, False, True)
                        
            if x=='v':
                print("OFF")
                Motor_off(p)
                
            if x=='r':
                print("run")
                if(temp1==1):
                 GPIO.output(in1,GPIO.HIGH)
                 GPIO.output(in2,GPIO.LOW)
                 print("forward")
                 x='z'
                else:
                 GPIO.output(in1,GPIO.LOW)
                 GPIO.output(in2,GPIO.HIGH)
                 print("backward")
                 x='z'


            elif x=='s':
                print("stop")
                GPIO.output(in1,GPIO.LOW)
                GPIO.output(in2,GPIO.LOW)
                x='z'

            elif x=='f':
                print("forward")
                GPIO.output(in1,GPIO.HIGH)
                GPIO.output(in2,GPIO.LOW)
                temp1=1
                x='z'

            elif x=='b':
                print("backward")
                GPIO.output(in1,GPIO.LOW)
                GPIO.output(in2,GPIO.HIGH)
                temp1=0
                x='z'

            elif x=='l':
                print("low")
                p.ChangeDutyCycle(25)
                x='z'

            elif x=='m':
                print("medium")
                p.ChangeDutyCycle(50)
                x='z'

            elif x=='h':
                print("high")
                p.ChangeDutyCycle(75)
                x='z'
             
            
            elif x=='e':
                GPIO.cleanup()
                print("GPIO Clean up")
                break
            
            else:
                print("<<<  wrong data  >>>")
                print("please enter the defined data to continue.....")



