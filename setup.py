import time
import RPi.GPIO as g

g.cleanup()
g.setmode(g.BCM)
g.setup(4,g.OUT)
g.setup(22,g.OUT)
g.setup(23,g.OUT)
g.setup(24,g.OUT)
g.setup(12,g.OUT)
g.setup(13,g.OUT)
g.setup(18,g.OUT)
g.setup(19,g.OUT)
pwm1 = g.PWM(18, 100)
pwm2 = g.PWM(19, 100)
pwm3 = g.PWM(13, 100)
pwm4 = g.PWM(12, 100)
pwm1.start(0)
pwm2.start(0)
pwm3.start(0)
pwm4.start(0)


def turn_bl(a,c):
	g.output(24,c)
	pwm2.ChangeDutyCycle(50)
	time.sleep(a)
	g.output(19,False)
	pwm2.ChangeDutyCycle(0)	

def turn_fr(a,c):
	g.output(4,c)
	pwm1.ChangeDutyCycle(50)
	time.sleep(a)
	g.output(19,False)
	pwm1.ChangeDutyCycle(0)

def turn_br(a,c):
	g.output(22,c)
	pwm3.ChangeDutyCycle(50)
	time.sleep(a)
	g.output(19,False)
	pwm3.ChangeDutyCycle(0)

def turn_fl(a,c):
	g.output(23,c)
	pwm4.ChangeDutyCycle(50)
	time.sleep(a)
	g.output(19,False)
	pwm4.ChangeDutyCycle(0)


def diff_turn(dir):
    g.output(23,dir)
    g.output(4,dir)
    g.output(22,not dir)
    g.output(24,not dir)
    # g.output(23,dir)
    # g.output(4,not dir)
    # g.output(22, dir)
    # g.output(24,not dir)
    pwm1.ChangeDutyCycle(50)
    pwm2.ChangeDutyCycle(50)
    pwm3.ChangeDutyCycle(50)
    pwm4.ChangeDutyCycle(50)
    time.sleep(4.5)
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)
    pwm3.ChangeDutyCycle(0)
    pwm4.ChangeDutyCycle(0)



def autonom_turn(a,c):
    g.output(23,c)
    g.output(4,c)
    pwm4.ChangeDutyCycle(50)
    pwm1.ChangeDutyCycle(50)
    time.sleep(a)
    pwm4.ChangeDutyCycle(0)
    pwm1.ChangeDutyCycle(0)