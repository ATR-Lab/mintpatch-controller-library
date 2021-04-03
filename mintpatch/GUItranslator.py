"""
Responsible for communication between the GUI and library.
Currently reads from JSON and outputs to consule.
Legacy version uses a python consule.
Original Author: Nathan Moder
Modified and cleaned up by: M. Arnett
3/29/2021
"""


# Imports for interaction with Node.js
import sys, json


# Uses funcitons from RobotManager and the wrapper, but does not need to import them.
class GUITranslator:
    def __init__(self,_manager):
        """
        Constructor. Just needs a RobotManager called _manager, 
        which the GUITranslator will access
        """
        # The RobotManager the translator will use.
        self.manager = _manager
        
        # An initially empty list of the servo names of all running motors
        # A servo name includes both the port and the ID, formatted:
        # <portname>_<motorID>
        self.running_motors = []


    def read_from_json(self):
        """
        Boiler-plate code for interacting with the commands sent from Node.js
        Returns a string
        """
        # jin becomes the next line in the JSON file created by the Node.
        # It needs to be translated
        jin = sys.stdin.readline()
        print(json.loads(jin))

        # loads() does the translation
        return json.loads(jin)


    def print_from_dict(self, idict):
        """
        An internal functions for readability. 
        Prints the info out of a Dictionary from the wrapper.
        Output, but no returns.
        """
        # idict is the information dictionary from a single motor.
        # Order to be printed:
        # id, stateID, voltage, temperature, angle, speed
        # The dictionary has a boolean value declaring whether the motor is moving.
        if idict['moving']:
            state = 'RUNNING'
        
        else:
            state = 'IDLE'
        
        # Prints the info out in one line as a formatted string.
        # Every element is seperated by a space.
        print('{id} {state} {voltage} {temp} {pos} {speed}'.format(id = idict["id"], 
        state = state, voltage = idict["voltage"], temp = idict["temperature"], 
        pos = {idict["position"]}, speed = idict["speed"]))    
        
    
    def stop_motor(self,servo_name):
        """
        Halts the movement of the named servo
        No returns or output
        """
        # These two variables can be adjusted if we allow 2-digit or 3-digit motor ID's.
        # The name of the port is the first part of the string.
        # We use python string operations to remove the last 2 characters.
        pname = servo_name[:-4]

        # The ID of the motor is just the last element.
        sid = int(servo_name[-3:])

        # Attepting to stop a motor that doesn't exist would be bad. Many errors.
        if self.manager.check_included(pname,sid):
            # Disable torque when stopping the motor
            self.manager.ports_by_name[pname].proxy.set_torque_enabled(sid, [0])

            # Uses a function in the wrapper to set the goal velocity to zero. 
            # This will slow the motor to a stop.
            # ports_by_name is indexed by the port name, 
            # while the wrapper function needs the motor ID.
            self.manager.ports_by_name[pname].proxy.set_goal_position(sid,0)

            # Removes the name from the internal list. 
            # It is no longer known to be running.
            self.running_motors.remove(servo_name)
                
        
        # This path depends on our safety priorities.
        # TODO: Determine if error correction will cause latency issues.
        """
        else:
            #if a servo is entered that doesn't seem to exist, we assume
            #that the typo was dangerous and stop the whole system
            self.stop_all_motors()
        """


    def stop_all_motors(self):
        """
        Halts all movement in the system.
        No returns or output.
        """
        # We loop though ever motor that we know to be running, and stop them.
        for motor in self.running_motors:
            self.stop_motor(motor)


    def move_motor(self, servo_name, speed):
        """
        Function member will enable torque, and then change the goal position,
        which then appends the servo to a list of running motors.
        Returns nothing.
        """
        #These values hold the same role as in stop_motor.
        pname = servo_name[:-4]
        sid = int(servo_name[-3:])

        # Trying to move a motor not attached to the system would be 
        # just as bad as stopping one.
        if self.manager.check_included(pname,sid):
            # Enable torque for motor to have it set goal position.
            self.manager.ports_by_name[pname].proxy.set_torque_enabled(sid, [1])

            # Set goal position of the motor
            self.manager.ports_by_name[pname].proxy.set_goal_position(sid,int(speed))

            # We add the motor to a list of currently running motors.
            self.running_motors.append(servo_name)


    def update_motor(self, servo_name):
        """
        Prints the full information set for one named servo.
        Output, but no returns.
        """
        # These values hold the same role as in stop_motor.
        pname = servo_name[:-4]
        sid = int(servo_name[-3:])

        # We cannot try to access a motor which isn't attached.
        if self.manager.check_included(pname, sid):
        
            # Defines an empty set which will store the information dictionary.
            idict = {}
        
            # Uses the wrapper to fill that dictionary with the needed entries.
            idict = self.manager.ports_by_name[pname].proxy.get_feedback(sid)
        
            # The motor ID in the dictionary does not include the port.
            # Since we want to output a combined name, we alter the entry.
            idict['id'] = servo_name

            # Since this was a dictionary returned from get_feedback, we can output it with
            # this internal function.
            self.print_from_dict(idict)

        else:
            print("No such servo")
        

    def update_all(self):
        """
        Function for updating the all the information on every servo motor.
        """
        # Since the RobotManager has the data structures which are needed
        # to access every servo, the data collection is handled by that class.
        # This function returns an array of dictionaries.
        all_info = self.manager.list_all_servo_info()
        
        # We loop through all the individual dictionaries, and print them out.
        for lmotor in all_info:
            self.print_from_dict(lmotor)

    
    def listen(self):
        """
        This is the function that MintPatch will be in while idle.
        Some Output is handled here.
        However, the main purpose of this function is to parse input and
        call the appropriate functions to handle the command cases.
        """
        
        # LOOP SETUP
        Continue=True

        # Establishes a common, potentially infinite while loop.
        # It can be ended by inputing "end".
        while(Continue):
            # GATHERING INPUT
            # NODE.JS
            # Uses this input function to gather one line from the Node.
            # This is a string. 
            # TODO: Implement this possibly
            #conin = self.read_from_json()        
            
            # CONSOLE
            conin = sys.stdin.readline()

            # INPUT SETUP
            # Since it's a string, and the parts of the input should be seperated
            # by spaces; we split this to make it an array.
            cinar = conin.split()
            
            # Knowing the length of this string array will
            # let us catch errors and quickly skip faulty commands.
            l = cinar.__len__()

            # HANDLE DIFFERENT COMMANDS
            # In this case, there is no input.
            # There's no reason to go through the loop.
            if l == 0:
                continue

            # In this case, we know we will be stopping motors.
            if cinar[0] == "stop":
                # If the word is alone, we assume this is an emergency stop.
                if l == 1:
                    self.stop_all_motors()

                # Otherwise, we let the user specify a single motor to stop.
                else:
                    # If they try to specify more than one motor, the format is wrong.
                    if l != 2:
                        continue

                    # Correctly called, we stop the motor.
                    self.stop_motor(cinar[1])

                continue

            if cinar[0] == "move":
                # We need both a servo and a speed, and nothing else.
                if l != 3:
                    continue

                self.move_motor(cinar[1], cinar[2])
                continue


            if cinar[0] == "update":
                if l == 1: 
                   continue

                self.update_motor(cinar[1])
                continue

            # Updates every running motor.
            if cinar[0] == "running":
                # Uses the update_motor function on every motor in the array.
                for motor in self.running_motors:
                    self.update_motor(motor)

                continue
            
            if cinar[0] == "list":
                for string in self.manager.list_servos():
                    print(string)

                continue

            if cinar[0] == "scan":
                self.update_all()
                continue

            if cinar[0] == "end":
                Continue = False

        # END LOOP
        print("Thank you for using MintPatch!")