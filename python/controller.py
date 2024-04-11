"""startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False

#========================
#========================
    # the functions

def setupSerial(baudRate, serialPortName):
    
    global  serialPort
    
    serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True)

    print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))

    waitForArduino()

#========================

def sendToArduino(stringToSend):
    
        # this adds the start- and end-markers before sending
    global startMarker, endMarker, serialPort
    
    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend
    stringWithMarkers += (endMarker)

    serialPort.write(stringWithMarkers.encode('utf-8')) # encode needed for Python3


#==================

def recvLikeArduino():

    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("utf-8") # decode needed for Python3
        
        if dataStarted == True:
            if x != endMarker:
                dataBuf = dataBuf + x
            else:
                dataStarted = False
                messageComplete = True
        elif x == startMarker:
            dataBuf = ''
            dataStarted = True
    
    if (messageComplete == True):
        messageComplete = False
        return dataBuf
    else:
        return "XXX" 

#==================

def waitForArduino():

    # wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded
    
    print("Waiting for Arduino to reset")
     
    msg = ""
    while msg.find("Arduino is ready") == -1:
        msg = recvLikeArduino()
        if not (msg == 'XXX'): 
            print(msg)



#====================
#====================
    # the program


setupSerial(115200, "/dev/ttyACM0")
count = 0
prevTime = time.time()
while True:
            # check for a reply
    arduinoReply = recvLikeArduino()
    if not (arduinoReply == 'XXX'):
        print ("Time %s  Reply %s" %(time.time(), arduinoReply))
        
        # send a message at intervals
    if time.time() - prevTime > 1.0:
        sendToArduino("this is a test " + str(count))
        prevTime = time.time()
        count += 1"""

"""import serial.tools.list_ports
import serial
import time
        
from inputs import get_gamepad
import math
import threading

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self): # return the buttons/triggers that you care about in this methode
        x = self.LeftJoystickX
        y = self.LeftJoystickY
        a = self.A
        b = self.X # b=1, x=2
        rb = self.RightBumper
        return [x, y, a, b, rb]


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state

if __name__ == '__main__':
    joy = XboxController()
    while True:
        print(joy.read())

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()

portsList = []

for onePort in ports:
    portsList.append(str(onePort))
    print(str(onePort))

val = input("Select Port: ")

portVar = None

for x in range(0, len(portsList)):
    if portsList[x].startswith(val):
        portVar = val
        print(portVar)

if portVar is None:
    print(f"Port {val} not found.")
    exit()

# Configurer le port série avant de l'ouvrir
serialInst.baudrate = 9600
serialInst.port = portVar

try:
    serialInst.open()
except serial.SerialException as e:
    print(f"Error opening the serial port: {e}")
    exit()

while True:
    command = input("Arduino Command: (ON/OFF): ")
    serialInst.write(command.encode('utf-8'))

    if command == 'exit':
        serialInst.close()  # Fermer le port série avant de quitter
        exit()"""

import inputs
import time

def check_xbox_controller_connection():
    # Recherche des événements de la manette Xbox
    devices = inputs.devices.gamepads
    xbox_controller_found = False

    for device in devices:
        print(f"Found Gamepad: {device.name}")
        if "Xbox" in device.name:
            xbox_controller_found = True
            break

    if xbox_controller_found:
        print("Xbox Controller Found. It is connected.")
    else:
        print("No Xbox Controller found. Please check the connection.")

if __name__ == "__main__":
    while True:
        check_xbox_controller_connection()
        time.sleep(5)

import pygame
import serial
import time

class XboxController(object):
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def read(self):
        x = self.joystick.get_axis(0)
        y = self.joystick.get_axis(1)
        a = self.joystick.get_button(0)
        b = self.joystick.get_button(1)
        rb = self.joystick.get_button(5)
        return x, y, a, b, rb

# Configuration du port série
serial_inst = serial.Serial()
serial_inst.baudrate = 9600
serial_inst.port = "/dev/tty.usbmodem142201"  # Remplacer par le bon port série

try:
    serial_inst.open()
except serial.SerialException as e:
    print(f"Error opening the serial port: {e}")
    exit()

# Fonction pour envoyer la commande à l'Arduino
def send_command(command):
    serial_inst.write(command.encode('utf-8'))

if __name__ == '__main__':
    joy = XboxController()

    led_status = False

    try:
        while True:
            x, y, a, b, rb = joy.read()

            # Contrôle de la LED en fonction du bouton A
            if a:
                if not led_status:
                    print("LED ON")
                    send_command("LED_ON")
                    led_status = True
            elif b:
                if led_status:
                    print("LED OFF")
                    send_command("LED_OFF")
                    led_status = False

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        serial_inst.close()
