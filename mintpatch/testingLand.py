"""
Temporary startup code. eventually we'll use a proper main
#Nathan Moder
#3/23/2021
"""

#TO MARCUS: look in PortManger to see the two lines that need changing

from modelTranslator import FakeMotor
from GUItranslator import GUITranslator
from robotManager import RobotManager

settings_set={}

#This on works with the fake servos included in the directory
#"""
port_list={'port_1','port_2'}
settings={
    'baudrate': 100,
    'minID': 1,
    'maxID': 50,
    'updateRate' : 100,
    'diagnosticsRate' : 100
}
settings_set['port_1']=settings
settings_set['port_2']=settings
"""
#This one is real, and matches the launch file simple_l_arm_controller_manager.launch
port_list={'ttyUSB0'}
settings={
    'baudrate': 1000000,
    'minID': 1,
    'maxID': 40,
    'updateRate' : 20,
    'diagnosticsRate' : 1
}
settings_set['ttyUSB0']=settings
"""


ze_manager=RobotManager(port_list,settings_set)
translator=GUITranslator(ze_manager)
translator.listen()