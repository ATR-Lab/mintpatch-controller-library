"""
Emulated motor setup.
Will be unused in final implementation.
Nathan Moder
3/29/2021
"""

#output order will be: id, name, stateID, voltage, temperature, angle, speed

#There will be three motors on two ports:
    #port_1:
        #motors on address 1 and 5
    #port2:
        #motor on address 1

import time

class EmulatedMotor:
    def __init__(self, _id, _stateID, _voltage, _temperature, _angle, _speed):
        self.id=_id
        self.stateID=_stateID
        self.voltage=_voltage
        self.temperature=_temperature
        self.angle=_angle
        self.speed=_speed
        self.goal_speed=0
        self.start_time=0
        self.moving=False
        self.last_check=0
    def changeName(self, new_name):
        self.name=new_name
    def set_goal_speed(self, new_speed):
        if new_speed==0:
            self.goal_speed=0
            self.speed=0
            self.moving=False
        else:
            self.goal_speed=new_speed
            self.start_time=time.time()
            self.moving=True
            self.last_check=self.start_time
    def check_while_running(self):
        moment=time.time()
        tchange=moment-self.start_time
        if tchange>15:
            self.speed=self.goal_speed
        else:
            self.speed=self.goal_speed/(15-tchange)
        self.angle=self.angle+(self.speed*6)*tchange
        while self.angle>360:
            self.angle=self.angle-360
        if self.moving==False:
            self.speed=0


global tm1
tm1=EmulatedMotor(1,"IDLE",50,60,0,0)
global tm2
tm2=EmulatedMotor(5,"IDLE",50,61,25,0)
global tm3
tm3=EmulatedMotor(1,"IDLE",70,65,10,0)
