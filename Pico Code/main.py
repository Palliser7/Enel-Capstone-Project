from machine import Pin, PWM, UART
import _thread
import time

#global variable for transferring data between threads
data = []

# Class for controlling the UART communication
class coms():
    def __init__(self, uartNum, baudRate, transmitPin, receivePin):
        self.uartNum = uartNum
        self.baudRate = baudRate
        self.transmitPin = transmitPin
        self.receivePin = receivePin
        self.uart = UART(uartNum, baudRate, tx=Pin(transmitPin), rx=Pin(receivePin))
        self.uart.init(bits = 8, parity = None, stop = 1)

    #writes a message to the uart tx pin
    def write(self, message):
        self.uart.write(message)

    #reads a message from the uart rx pin
    def read(self):
        if self.uart.any():
            return self.uart.read()
    
    #encodes a message to utf-8
    def encode(self, message):
        return message.encode('utf-8')
    
    #decodes a message from utf-8
    def decode(self, message):
        return message.decode('utf-8')
    

# Class for controlling a single coil
class coilDriver():
    def __init__(self, pos, neg):
        self.pwmPos = PWM(Pin(pos))    #initializing pwm on the positive pin
        self.pwmNeg = PWM(Pin(neg))    #initializing pwm on the negative pin
        self.LUT = [32768,39160,45307,50972,55938,60013,63041,64905,65535,64905,63041,60013,55938,50972,45307,39160,32768,39160,45307,50972,55938,60013,63041,64905,65535,64905,63041,60013,55938,50972,45307,39160]
        self.LUT2 = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]    #positive of negative identifier LUT
        self.modifiedLUT = self.LUT            #LUT that will be modified by phase shift
        self.modifiedLUT2 = self.LUT2          #LUT2 that will be modified by phase shift
        self.pwmPos.freq(960)                  #frequency of positive pwm drive
        self.pwmNeg.freq(960)                  #frequency of negative pwm drive
        self.pwmPos.duty_u16(0)                #duty cycle of positive pwm drive
        self.pwmNeg.duty_u16(0)                #duty cycle of negative pwm drive
        self.phase = 0                         #value of current phase shift from original signal in degrees
        self.amplitude = 0.5                   #multiplier for the amplitude of the wave (0-1), starts at 50% to limit initial current draw
    
    def changeDuty(self, duty, sign):
        if sign == 0:
            self.pwmNeg.duty_u16(0)                             #changes the duty cycle of the negative pwm drive to 0 to stop shorting the circuit
            time.sleep(0.000001)
            self.pwmPos.duty_u16(int(self.amplitude * duty))    #changes the duty cycle of the positive pwm drive
        elif sign == 1:
            self.pwmPos.duty_u16(0)                             #changes the duty cycle of the positive pwm drive to 0 to stop shorting the circuit
            time.sleep(0.000001)
            self.pwmNeg.duty_u16(int(self.amplitude * duty))    #changes the duty cycle of the negative pwm drive
    
    #takes the phase shift in degrees and changes the LUT accordingly
    #we have to take the phase shift rounded to the nearest 11.25 degrees
    #because the LUT is only 32 values long
    #then we shift the LUT by the number of values = rounded phase shift/11.25
    def changePhase(self, phase):
        shift = (phase//(11.25))
        self.phase = (shift*11.25)
        self.modifiedLUT = self.LUT[shift:] + self.LUT[:shift]
        self.modifiedLUT2 = self.LUT2[shift:] + self.LUT2[:shift]

    #emergency stop to kill all pwm outputs
    def Kill(self):
        self.pwmPos.duty_u16(0)
        self.pwmNeg.duty_u16(0)
        self.pwmPos.freq(0)
        self.pwmNeg.freq(0)
           
# parses the revieved data into a direction and position
def parseData(data):
    direction = ("x", -30)
    position = ("x", "y")
    return direction, position

def communication():
    #initialize the uart pins
    uart = coms(0, 9600, 1, 2)

    while 1:
        recievedMsg = uart.read()
        if recievedMsg != None:
            newData = uart.decode(recievedMsg)
            data = parseData(newData)
        recievedMsg = ''
        time.sleep(0.1)

# Function for calculating the phase shift and amplitude of each coil
def calculate(direction, position):
    phase1 = 0
    amplitude1 = 0
    phase2 = 0
    amplitude2 = 0
    phase3 = 0
    amplitude3 = 0
    c1 = (phase1, amplitude1)
    c2 = (phase2, amplitude2)
    c3 = (phase3, amplitude3)
    return c1, c2, c3

def main():
    #initialize the 3 coils
    coil1 = coilDriver(16, 17)
    coil2 = coilDriver(18, 19)
    coil3 = coilDriver(20, 21)
    prevData = ''
    #starts the communication thread
    _thread.start_new_thread(communication, ())

    while(1):
        newData = data

        if newData == 'esc':
            coil1.Kill
            coil2.Kill
            coil3.Kill
            break

        if newData != prevData:
            c1, c2, c3 = calculate(newData[0], newData[1])
            coil1.changePhase(c1[0])
            coil1.amplitude = c1[1]
            coil2.changePhase(c2[0])
            coil2.amplitude = c2[1]
            coil3.changePhase(c3[0])
            coil3.amplitude = c3[1]

            for i in range(32):
                coil1.changeDuty(coil1.modifiedLUT[i], coil1.modifiedLUT2[i])
                coil2.changeDuty(coil2.modifiedLUT[i], coil2.modifiedLUT2[i])
                coil3.changeDuty(coil3.modifiedLUT[i], coil3.modifiedLUT2[i])
                time.sleep(0.001)

        else:
            for i in range(32):
                coil1.changeDuty(coil1.modifiedLUT[i], coil1.modifiedLUT2[i])
                coil2.changeDuty(coil2.modifiedLUT[i], coil2.modifiedLUT2[i])
                coil3.changeDuty(coil3.modifiedLUT[i], coil3.modifiedLUT2[i])
                time.sleep(0.001)
        
        prevData = newData