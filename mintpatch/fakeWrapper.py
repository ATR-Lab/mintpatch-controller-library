"""
This is all purely test
"""
from modelTranslator import FakeMotor
from modelTranslator import tm1
from modelTranslator import tm2
from modelTranslator import tm3

class SDKSerialWrapper:
    def __init__(self, port, baudrate, feedback_echo=False):
        self.port=port
        #print("wrapper init")

    def read(self, servo_id, address, size):
        print("wrapper read")
    def write(self, servo_id, address, data):
        print("wrapper write")
    def sync_write(self, address, data):
        print("wrapper syncwrite")
    
    def set_goal_velocity(self, servo_id, goal):
        
        #print(self.port)
        if self.port=="/dev/port_1":
            #print(servo_id)
            if 1==int(servo_id):
                #print("into if")
                tm1.set_goal_speed(goal)
            elif int(servo_id)==5:
                tm2.set_goal_speed(goal)
        elif self.port=="/dev/port_2":
            if int(servo_id)==1:
                tm3.set_goal_speed(goal)

    def get_feedback(self,servo_id):
        if self.port=="/dev/port_1":
            if int(servo_id)==1:
                if tm1.moving:
                    tm1.check_while_running()
                return {
                    'id':tm1.id,
                    'goal': 0,
                    'position': tm1.angle,
                    'error' : 0,
                    'speed' : tm1.speed,
                    'load' : 0,
                    'voltage' : tm1.voltage,
                    'temperature' : tm1.temperature,
                    'moving' : tm1.moving
                }
            if int(servo_id)==5:
                if tm2.moving:
                    tm2.check_while_running()
                return {
                    'id':tm2.id,
                    'goal': 0,
                    'position': tm2.angle,
                    'error' : 0,
                    'speed' : tm2.speed,
                    'load' : 0,
                    'voltage' : tm2.voltage,
                    'temperature' : tm2.temperature,
                    'moving' : tm2.moving
                }
        if self.port=="/dev/port_2":
            if int(servo_id)==1:
                if tm3.moving:
                    tm3.check_while_running()
                return {
                    'id':tm3.id,
                    'goal': 0,
                    'position': tm3.angle,
                    'error' : 0,
                    'speed' : tm3.speed,
                    'load' : 0,
                    'voltage' : tm3.voltage,
                    'temperature' : tm3.temperature,
                    'moving' : tm3.moving
                }
        return 'no such servo'

        
