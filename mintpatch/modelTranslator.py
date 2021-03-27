#A functional, standalone testing environment for use in intial GUI testing.
#Nathan Moder
#3/21/2021

#output order will be: id, name, stateID, voltage, temperature, angle, speed


#There will be three motors on two ports:
    #port_1:
        #port_1_1, Test_Motor_1, 
    #port2:

import time



class ModelGUITranslator:
    def __init__(self):
        self.motors={}
        #self.motors={tm1,tm2,tm3}
        print("MintPatch is ready")
    def update_running(self):
        for motor in self.motors:
            if motor.stateID=="RUNNING":
                motor.classPrint()
        
    def update_all(self):
        for motor in self.motors:
            motor.classPrint()
        
    def add_scan(self):
        self.update_all()
        
    def listen(self):
        cont=True
        while(cont):
            conin=input()
            cinar=conin.split()
            #diagnostic for loop
            l=cinar.__len__()
            """
            An if-else chain will NOT be used in the final version.
            It is used here to expediate testing
            """
            if l==0:
                continue #I'm sorry my teachers
            if cinar[0]=="update":
               if l==1: 
                   continue

            if cinar[0]=="scan":
                self.add_scan()
            if cinar[0]=="end":
                cont=False

            #TODO: Learn python switch theory
            #TODO: Learn lambda and function objects for python
            """
            cases:
            end: "disconnect"
            scan: "add_scan"
            running: "update_running"
            all: "update_all"
            update <servoID>: "update_single_servo"
            speed <servoID>: "
            set <attribute> <servoID>
            run <servoID>
            stop <servoID>
            """
            #this is the function that MintPatch will be in while idle.
            #we need to establish a communication protocol for
            #the python command line.

class FakeMotor:
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
        #print("into fake servo move")
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


#just starts up the translator
"""
global tm1
tm1=FakeMotor(1,"IDLE",50,60,0,0)
global tm2
tm2=FakeMotor(2,"IDLE",50,61,25,0)
global tm3
tm3=FakeMotor(1,"IDLE",70,65,10,0)
"""
global tm1
tm1=FakeMotor(1,"IDLE",50,60,0,0)
global tm2
tm2=FakeMotor(5,"IDLE",50,61,25,0)
global tm3
tm3=FakeMotor(1,"IDLE",70,65,10,0)
#translator=GUITranslator()
#translator.update_all()
#translator.listen()
