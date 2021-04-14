"""
The startup code for MintPatch.
Nathan Moder & M. Arnett
3/29/2021
"""


# Imports from custom classes
from MINTpatchServer import ServerManager
from robotManager import RobotManager
from emulatedMotor import EmulatedMotor

# Import standard libraries
import json, sys, os

#Imports zerorpc to interact with clients
import zerorpc

def main():
    """
    # Setttings for configuring our pinging to the motors.
    settings_set = {}

    # Get the port name through a system call
    raw_port_command = os.popen("ls /dev/ttyUSB*")
    raw_ports = raw_port_command.read()
    port_list_raw = raw_ports.split()
    port_list = []

    # Real motors, and matches the launch file simple_l_arm_controller_manager.launch
    # port_list = {port}
    settings = {
        'baudrate': 1000000,
        'minID': 1,
        'maxID': 40,
        'updateRate' : 20,
        'diagnosticsRate' : 1
    }

    # Get Port List without /dev/
    for raw_port in port_list_raw:
        port = raw_port.replace('/dev/', '')
        settings_set[port] = settings
        port_list.append(port)
    """

    #This seperate code with the emulated servos included in the directory.
    #Comment it out when using real motors
    """
    DO NOT REMOVE THIS CODE! Our team has members without constant access to real motors.
    """
    #"""
    settings_set = {}
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
    #"""

    # Making robot manager, and basically listening in on the GUI translator
    manager = RobotManager(port_list, settings_set)
    translator = zerorpc.Server(ServerManager(manager))
    translator.bind("tcp://0.0.0.0:4242")
    translator.run()


if __name__ == '__main__':
    main()