from machine import Pin, PWM
import time

def init():
    #initializing pwm on pin 16
    coil1pos = PWM(Pin(16))
    coil1pos.freq(960)
    coil1pos.duty_u16(0)
    
    #initializing pwm on pin 17
    coil1neg = PWM(Pin(17))
    coil1neg.freq(960)
    coil1neg.duty_u16(0)
    
    #initializing pwm on pin 18
    coil2pos = PWM(Pin(18))
    coil2pos.freq(960)
    coil2pos.duty_u16(0)
    
    #initializing pwm on pin 19
    coil2neg = PWM(Pin(19))
    coil2neg.freq(960)
    coil2neg.duty_u16(0)
    
    #initializing pwm on pin 20
    coil3pos = PWM(Pin(20))
    coil3pos.freq(960)
    coil3pos.duty_u16(0)
    
    #initializing pwm on pin 21
    coil3neg = PWM(Pin(21))
    coil3neg.freq(960)
    coil3neg.duty_u16(0)
    
    
    #changes the duty cycle of the coil pwm drives
    #coil signifies the coil number(1 = inner, 2 = middle, 3 = outer)
    #duty signifies the duty cycle that the pwm will change too
    #side signifies either the positive or negative drive pwm(1 = positive, 2 = negative) 
def changeDuty(coil, duty, side):
    coil1pos = PWM(Pin(16))
    coil1neg = PWM(Pin(17))
    coil2pos = PWM(Pin(18))
    coil2neg = PWM(Pin(19))
    coil3pos = PWM(Pin(20))
    coil3neg = PWM(Pin(21))
    if coil == 1:
        if side == 1:
            coil1pos.duty_u16(int(duty))
        elif side == 2:
            coil1neg.duty_u16(int(duty))
    elif coil == 2:
        if side == 1:
            coil2pos.duty_u16(int(duty))
        elif side == 2:
            coil2neg.duty_u16(int(duty))
    elif coil == 3:
        if side == 1:
            coil3pos.duty_u16(int(duty))
        elif side == 2:
            coil3neg.duty_u16(int(duty))
        


LUT = [32768,39160,45307,50972,55938,60013,63041,64905,65535,64905,63041,60013,55938,50972,45307,39160,32768,39160,45307,50972,55938,60013,63041,64905,65535,64905,63041,60013,55938,50972,45307,39160]
while(1):
    init()
    count = 0
    z = 0
    y = 0
    for x in LUT:
        if count > 16:
            changeDuty(1,0,1)
            time.sleep(0.0001)
            changeDuty(1,x,2)
        else:
            changeDuty(1,0,2)
            time.sleep(0.0001)
            changeDuty(1,x,1)
        time.sleep(0.002)
        count += 1
        
# def phase_shift(coil, phase):
#     if coil == 1:
#         coil3pos = PWM(Pin(16))
#         coil3neg = PWM(Pin(17))
#     elif coil == 2:
#         coil3pos = PWM(Pin(18))
#         coil3neg = PWM(Pin(19))
#     elif coil == 3:
#         coil3pos = PWM(Pin(20))
#         coil3neg = PWM(Pin(21))