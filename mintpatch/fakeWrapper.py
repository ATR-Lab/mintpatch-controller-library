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
    
    def get_feedback(self,servo_id):
        if self.port=="/dev/port_1":
            if servo_id==1:
                #return tm1.classString()
                return {
                    'id':tm1.id,
                    'goal': 0,
                    'position': tm1.angle,
                    'error' : 0,
                    'speed' : tm1.speed,
                    'load' : 0,
                    'voltage' : tm1.voltage,
                    'temperature' : tm1.temperature,
                    'moving' : False
                }
            if servo_id==5:
                #return tm2.classString()
                return {
                    'id':tm2.id,
                    'goal': 0,
                    'position': tm2.angle,
                    'error' : 0,
                    'speed' : tm2.speed,
                    'load' : 0,
                    'voltage' : tm2.voltage,
                    'temperature' : tm2.temperature,
                    'moving' : False
                }
        if self.port=="/dev/port_2":
            if servo_id==1:
                #return tm3.classString()
                return {
                    'id':tm3.id,
                    'goal': 0,
                    'position': tm3.angle,
                    'error' : 0,
                    'speed' : tm3.speed,
                    'load' : 0,
                    'voltage' : tm3.voltage,
                    'temperature' : tm3.temperature,
                    'moving' : False
                }
        return 'no such servo'

        