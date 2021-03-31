"""
The startup code for MintPatch.
Temporarily, there is no input.
TODO: There will eventually be dynamic startup.
#Nathan Moder
#3/29/2021
"""

from emulatedMotor import EmulatedMotor
from GUItranslator import GUITranslator
from robotManager import RobotManager
import json, sys


def main():
    settings_set={}
    
    #This one works with the emulated servos included in the directory
    """
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
    # """

    manager=RobotManager(port_list,settings_set)
    translator=GUITranslator(manager)
    translator.listen()

if __name__ == '__main__':
    main()
