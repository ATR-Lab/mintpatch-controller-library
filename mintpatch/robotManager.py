"""
Creates the Servo Manager, which will store information about
which port each servo is attached to.
It also handles the startup of MintPatch.
Author: Nathan Moder
3/29/2021
"""


# Importing the port manager
from portManager import PortManager


class RobotManager:
    def __init__(self, _port_list, _setup_info):
        """
        Constructor. _port_list is an array of strings; 
        this constructor will set up every port manager.
        _setup_info is a dictionary of settings that are used for the ports; 
        it is indexed by port name.
        After execution, the manager and all port managers will be ready to function.

        TODO: Implement code to get rid of empty portManagers, 
        if we'll be checking every port of the appropriate type
        """
        # Defines empty arrays
        # Array of pairs of (port_name(string), servo_id(int))
        self.servo_list = []

        # A dictionary that connects the PortManager objects to strings of their names
        self.ports_by_name = {}

        # String array of the paths to the ports
        self.port_names = _port_list   

        # For every port listed in the arguments, it makes a
        # port manager. Then, it makes not of every servo attached to it.
        for portn in _port_list:
            # Enter the loop for every port name.
            temp_manager = PortManager(portn,_setup_info[portn])
            
            # We want to make sure we aren't storing empty ports.
            if not temp_manager.is_empty():

                # For this port, a Port Manager is set up.
                # Since _setup_info is indexed by port name, we simply index it
                # The same name will is used to index ports_by_name 
                self.ports_by_name[portn] = temp_manager
                                 
            else:
                # If it is empty, we have no need to go into the other loop.
                continue

            # Utilizes and looks through the list of servos in the port manager.
            for servo in self.ports_by_name[portn].servos:

                # Stores a pair of values in the servo_list.
                # We want it to seperately track the port and servo ID for later ease.
                self.servo_list.append((portn, servo))


    def check_included(self, pname, sid):
        """
        Simple seach through the servo list to see if it exists.
        Could be used for preventing errors with faulty indexes
        and tool calls.
        Returns a boolean.
        """
        # Uses itterate and keep track algorithm
        found = False

        for pair in self.servo_list:
            # As this is a pair of port name and servo ID, we check both to see
            # if we found our matching servo.
            if pair[0] == pname and int(pair[1]) == int(sid):
                found = True
       
        return found
    

    def list_servos(self):
        """
        Returns the string list of every servo's name
        """
        # Initialize names list
        names = []

        # Check our servo list
        for pair in self.servo_list:
            psid = pair[1]
            sid = ''

            # If ID is 0-9, concatonate two zeros
            if psid < 10:
                sid = '00'+str(psid)

            # If ID is 10-99, concatonate one zero
            elif psid < 100:
                sid = '0'+str(psid)

            # Just concatonate the ID if 100-256
            else:
                sid = str(psid)

            # Append the string ID to the names list
            # e.g., ttyUSB0_014
            names.append('{pname}_{sid}'.format(pname = pair[0], sid = sid))

        return names


    def list_all_servo_info(self):
        """
        goes through every servo and gets info
        uses PortManager and Wrapper
        returns an array of dictionaries
        """
        # Defines the empty array which will hold the dictionaries
        info_dict = []
        
        # Loops through every Port Manager using their name,
        # since that is how they are stored
        for portn in self.port_names:
            port = self.ports_by_name[portn]

            # This loop goes through the servos on the port. 
            # ids is an integer number that corresponds
            # to the value in a register on the motor.
            for ids in port.servos:
                # Accesses the wrapper held by the port manager.
                # From it, it uses get_feedback to obtain a comprehensive
                # list of dynamic information from the servo.
                # TODO: Add somewhere a means of obtaining static info as well.
                temp_dict = port.proxy.get_feedback(ids)
                sid = ''

                if ids < 10:
                    sid = '00'+str(ids)

                elif ids < 100:
                    sid = '0'+str(ids)

                else:
                    sid = str(ids)

                # The ID returned only describes the motor, but we need to
                # provide the port name for it to be useful to outside systems.
                # We use the format function to create our new Servo Name:
                # <port_name>_<motor_id>
                # For example, for motor 5 on the port "USB0", we would have:
                # ttyUSB0_005
                # TODO: Must currently be adapted if we want double-digit motor ID's
                temp_dict['id'] = '{portn}_{id}'.format(portn = portn, id = sid)

                # We add the obtained and edited dictionary to the list we will return.
                info_dict.append(temp_dict)

        return info_dict


    def list_ports(self):
        """
        Returns an array of the names of every port.
        """
        return self.port_names
