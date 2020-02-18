
# Javier Rubia
# Code to pass rpm to the Odrive and its proper motor
#=======================================================
#/-- imports --/
import serial
import math

#/-- Global vars --/
cpr=2048 # number of counts per revolution (4*ppr)

#/-- functions --/
def writemotor(radsf,motor):

    veloc=(radsf*cpr)/(2*math.pi)# compute the rad/s through  counts/s
    print("vel : ",veloc )
    cad= "v "+str(motor)+" "+str(veloc)+" \r\n"
    cad=bytes(cad,encoding='utf8')

    print("final cad: ",cad)
    odrv0.write(cad)
    #listen the odrv back
    msg = odrv0.readline()
    if len(msg)>0:
        print("Error msg: ",msg)
    else:
        print("Msg send",cad)
    return

def readmotor(motor):

    cad= "f "+str(motor)+" \r\n"
    cad=bytes(cad,encoding='utf8')
    print("final cad: ",cad)
    odrv0.write(cad)

    #listen the odrv back
    msg = odrv0.readline()

    # process the info from the serial
    msg=msg.decode("utf-8")# decode
    msg=str(msg)# string
    msg=msg.split()# parse the information [0]=counts [1]counts/s
    msg=msg[1].replace("'","")# clean the extra '
    msg=float(msg)

    #process the info and change counts/s --> rad/s
    data=(msg/cpr)*2*math.pi# data=rev/s
    return msg

#/-- open Serial ports

odrv0 = serial.Serial("/dev/ttyACM0",115200,timeout=.3)# motors 1, 2
#odrv1 = serial.Serial("/dev/ttyACM1",115200,timeout=1)# motors 2, 3


#/-- main program --/
while True:
    # vars that will chose wich motor move and speed
    var=input("read(r) or write(w)?: ")
    motor=int(input("motor 0 o 1: "))
    if var=='w':
        rads=float(input("entra rad/s: "))
        writemotor(rads,motor)# obtain the proper cad to send through serial
    if var=='r':
        msg=readmotor(motor)# obtain the proper cad to send through serial
        print("motor:",motor," goes to rev/s:",msg)
    #/--
