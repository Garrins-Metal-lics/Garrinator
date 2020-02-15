
# Javier Rubia
# Code to pass rpm to the Odrive and its proper motor
#=======================================================
#/-- imports --/
import serial
import time

#/-- Global vars --/
cpr=2048 # number of counts per revolution


#/-- functions --/
def transcriptor(rpmf,motor):

    veloc=(rpmf*cpr)/60;# compute the rev/s through  counts/s
    print("vel : ",veloc )
    cad= "v "+str(motor)+" "+str(veloc)+" \r\n"
    cad=bytes(cad)
    return cad


#/-- open Serial ports

odrv0 = serial.Serial("/dev/ttyACM0",115200,timeout=1)# motors 1, 2
#odrv1 = serial.Serial("/dev/ttyACM1",115200,timeout=1)# motors 2, 3


#/-- main program --/
# vars that will chose wich motor move and speed
motor=0
while True:
    rpm=int(input("entra rpm: "))

    #/--
    vel=transcriptor(rpm,motor)# obtain the proper cad to send through serial
    print("final cad: ",vel)
    odrv0.write(vel)

    #listen the odrv back
    msg = odrv0.readline()
    print("the message read: ",msg)#show the message
