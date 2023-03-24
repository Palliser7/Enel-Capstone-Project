from machine import Pin, PWM
from machine import UART
import sys
import math
import _thread
import time

# #global variable for transferring data between threads
global data
data = ''

# # Class for controlling the UART communication
class coms():
    def __init__(self, uartNum, baudRate, transmitPin, receivePin):
        self.uartNum = uartNum
        self.baudRate = baudRate
        self.transmitPin = transmitPin
        self.receivePin = receivePin
        self.uart = UART(uartNum, baudRate, tx=Pin(transmitPin), rx=Pin(receivePin))
        self.uart.init(bits = 8, parity = None, stop = 1, timeout = 1)

    #writes a message to the uart tx pin
    def write(self, message):
        self.uart.write(message)

    #reads a message from the uart rx pin
    def read(self):
        if self.uart.any():
            return self.uart.readline()
 
    #encodes a message to utf-8
    def encode(self, message):
        return message.encode('utf-8')
    
    #decodes a message from utf-8
    def decode(self, message):
        return message.decode('utf-8')
    

# Class for controlling a single coil
class coilDriver():
    def __init__(self, pos, neg, p1, p2):
        self.pinP = pos
        self.pinN = neg
        self.pwmPos = PWM(Pin(pos))                             #initializing pwm on the positive pin
        self.pwmPos.deinit()
        self.pwmNeg = PWM(Pin(neg))                             #initializing pwm on the negative pin
        self.pwmNeg.deinit()
        self.P1 = Pin(p1, mode = Pin.OUT, value = 0)            #initializing pin for positive pwm drive
        self.P2 = Pin(p2, mode = Pin.OUT, value = 0)            #initializing pin for negative pwm drive
        self.LUT = [32768,39160,45307,50972,55938,60013,63041,64905,65535,64905,63041,60013,55938,50972,45307,39160,32768,32768,39160,45307,50972,55938,60013,63041,64905,65535,64905,63041,60013,55938,50972,45307,39160,32768]
        self.LUT2 = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]    #positive of negative identifier LUT
        self.modifiedLUT = self.LUT                             #LUT that will be modified by phase shift
        self.modifiedLUT2 = self.LUT2                           #LUT2 that will be modified by phase shift
        self.pwmPos.freq(960)                                   #frequency of positive pwm drive
        self.pwmNeg.freq(960)                                   #frequency of negative pwm drive
        self.pwmPos.duty_u16(0)                                 #duty cycle of positive pwm drive
        self.pwmNeg.duty_u16(0)                                 #duty cycle of negative pwm drive
        self.phase = 0                                          #value of current phase shift from original signal in degrees
        self.amplitude = 1                                      #multiplier for the amplitude of the wave (0-1), starts at 50% to limit initial current draw
        #changes the duty while grounding both sides of the coil to eliminate a floating reference across it
    
    def initpwm(self, x):
        if x == 0:
            self.pwmPos = PWM(Pin(self.pinP))                             #initializing pwm on the positive pin
            self.pwmPos.freq(960)
            self.pwmPos.duty_u16(0)
        else:
            self.pwmNeg = PWM(Pin(self.pinN))								#initializing pwm on the negative pin
            self.pwmNeg.freq(960)
            self.pwmNeg.duty_u16(0) 
            
    def changeDuty(self, duty, sign, prev):
        if sign == 0 and prev == 1:
            self.P1.value(0)									#changes the value of the negative pwm drive to 0 to prevent shorting the circuit
            self.pwmPos.deinit()
            a = Pin(self.pinP, mode = Pin.OUT, value = 1)
            time.sleep_us(500)
            self.pwmNeg.deinit()
            b = Pin(self.pinN, mode = Pin.OUT, value = 1)
            time.sleep_us(500)
            a.value(0)
            time.sleep_us(500)
            b = Pin(self.pinN, mode = Pin.OUT, value = 0)
            self.initpwm(1)
            self.pwmNeg.duty_u16(duty)
            self.P2.value(1)
        elif sign == 1 and prev == 0:
            self.P2.value(0)									#changes the value of the positive pwm drive to 0 to prevent shorting the circuit
            self.pwmNeg.deinit()
            a = Pin(self.pinN, mode = Pin.OUT, value = 1)
            time.sleep_us(500)
            self.pwmPos.deinit()
            b = Pin(self.pinP, mode = Pin.OUT, value = 1)
            time.sleep_us(500)
            a.value(0)
            time.sleep_us(500)
            b = Pin(self.pinP, mode = Pin.OUT, value = 0)
            self.initpwm(0)
            self.pwmPos.duty_u16(duty)
            self.P1.value(1)
        elif sign == 1:
            self.pwmPos.duty_u16(duty)
        elif sign == 0:
            self.pwmNeg.duty_u16(duty)
    
    # Takes the phase shift in degrees and changes the LUT accordingly
    # We have to take the phase shift rounded to the nearest 11.25 degrees
    # Because the LUT is only 32 values long
    # Then we shift the LUT by the number of values = rounded phase shift/11.25
    def changePhase(self, phase):
        shift = int(((phase)//(10.59)))
        self.phase = (shift*10.588)
        self.modifiedLUT = self.LUT[shift:] + self.LUT[:shift]
        self.modifiedLUT2 = self.LUT2[shift:] + self.LUT2[:shift]

    # Emergency stop to kill all pwm outputs
    def Kill(self):
        self.P1.value(0)
        self.P2.value(0)
        time.sleep_us(1000)
        self.pwmPos.duty_u16(65545)
        self.pwmNeg.duty_u16(65545)
        time.sleep_us(1000)
        self.pwmPos.duty_u16(0)
        self.pwmPos.deinit()
        self.pwmNeg.duty_u16(0)
        self.pwmNeg.deinit()
        print("killed pwm")
           
# Parses the revieved data into a direction and position
def parseData(data):
    direction = ("x", -30)
    position = ("x", "y")
    return direction, position

def communication():
    # Initialize the uart pins
    from machine import UART
    uart1 = coms(0, 9600, 0, 1)
    global data
    # Constantly check for new data from the Raspberry Pi
    while 1:
        recievedMsg = uart1.read()
        if recievedMsg != None:
            newData = recievedMsg
            print(recievedMsg)
            data = uart1.decode(recievedMsg)
        recievedMsg = ''
        time.sleep_us(100)

# Function for calculating the phase shift and amplitude of each coil
def calculate(direction, position):
    # Define constants
    mu0 = 4 * np.pi * 10**-7
    N1= 350 # Number of turns in inner coil
    N2 = 412 # Number of turns in middle coil
    N3 = 409 # Number of turns in outer coil
    NAvg = (N1+N2+N3)/3 # Average number of turns
    R1 = 0.0868 # Radius of inner coil
    R2 = 0.1268 # Radius of middle coil
    R3 = 0.1688 # Radius of outer coil

    K = 0.8 # Constant depending on magnetic properties of the conductive object

# Define the desired force and distance to the conductive object
    F_desired = direction # Newtons
    d = position # meters

# Set initial guesses for the currents
    I1 = 1 # Amperes
    I2 = 1 # Amperes
    I3 = 1 # Amperes

# Define a tolerance for convergence
    tolerance = 0.000001 # Newtons

# Start the iterative solver
    while True:
    # Calculate the magnetic field strengths for each coil
        B1 = mu0 * N1 * I1 / (2 * R1)
        B2 = mu0 * N2 * I2 / (2 * R2)
        B3 = mu0 * N3 * I3 / (2 * R3)
    
    # Calculate the force on the conductive object
        F_calculated = NAvg * K * B1 * B2 * B3 / d**2
    
    # Check for convergence
        if abs(F_calculated - F_desired) < tolerance:
            break
    
    # Adjust the currents based on the difference between the calculated and desired forces
        I1 += (F_desired - F_calculated) / (N1 * K * B2 * B3 / d**2)
        I2 += (F_desired - F_calculated) / (N2 * K * B1 * B3 / d**2)
        I3 += (F_desired - F_calculated) / (N3 * K * B1 * B2 / d**2)

    # Return final current values
    return I1, I2, I3

# Starts the communication thread
# _thread.start_new_thread(communication, ())
try:
    # Data Initialization
    global data
    newData = data
    prevData = ''
    data = ''
    # Initialize the coils
    coil1 = coilDriver(2, 3, 5, 7)
    coil2 = coilDriver(4, 6, 8, 9)
    coil3 = coilDriver(21, 22, 26, 20)
    
    prev1 = 0
    prev2 = 0
    prev3 = 0
    while True:
#         while True:
#             coil1.P1.value(0)
#             time.sleep_us(500)
#             coil1.P1.value(1)
#             time.sleep_us(500)
        # Checks for a kill message
        if newData == 'esc':
            coil1.Kill
            coil2.Kill
            coil3.Kill
            sys.exit()
        
        # Changes the sinewave data 
        if newData != prevData or '':
            c1, c2, c3 = calculate(newData[0], newData[1])
            coil1.changePhase(c1[0])
            coil1.amplitude = c1[1]
            coil2.changePhase(c2[0])
            coil2.amplitude = c2[1]
            coil3.changePhase(c3[0])
            coil3.amplitude = c3[1]
            
        # Runs the sinewave sequences through the coils
        for i in range(34):
            coil1.changeDuty(coil1.modifiedLUT[i], coil1.modifiedLUT2[i], prev1)
            coil2.changeDuty(coil2.modifiedLUT[i], coil2.modifiedLUT2[i], prev2)
            coil3.changeDuty(coil3.modifiedLUT[i], coil3.modifiedLUT2[i], prev3)
            prev1 = coil1.modifiedLUT2[i]
            prev2 = coil2.modifiedLUT2[i]
            prev3 = coil3.modifiedLUT2[i]
            time.sleep_us(1625)


except KeyboardInterrupt:
    # Kills all the coil drivers and exits
    coil1.Kill()
    coil2.Kill()
    coil3.Kill()
    sys.exit()