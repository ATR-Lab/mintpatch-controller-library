"""
The startup code for MintPatch.
Original Author: Nathan Moder
Cleaned up by: M. Arnett
3/29/2021
"""


# Imports from custom classes
from GUItranslator import GUITranslator
from robotManager import RobotManager

# Import standard libraries
import json, sys


def main():
    # Setttings for configuring our pinging to the motors.
    settings_set = {}

    # Real motors, and matches the launch file simple_l_arm_controller_manager.launch
    port_list = {'ttyUSB0'}
    settings = {
        'baudrate': 1000000,
        'minID': 1,
        'maxID': 40,
        'updateRate' : 20,
        'diagnosticsRate' : 1
    }
    settings_set['ttyUSB0'] = settings

    # Making robot manager, and basically listening in on the GUI translator
    manager = RobotManager(port_list, settings_set)
    translator = GUITranslator(manager)
    translator.listen()


if __name__ == '__main__':
    main()