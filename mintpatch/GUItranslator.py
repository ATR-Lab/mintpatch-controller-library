"""
Responsible for communication between the GUI and library.
Currently reads from JSON and outputs to console.
Test version uses a python console.
Author: Nathan Moder
Assistance: M. Arnett
4/6/2021
"""


# Imports for interaction with Node.js
import sys, json


class ServoLog:
    def __init__(self,servo_name):
        self.output_name=servo_name
        self.info_dictionary={}

    def log_servo(self, info_dictionary):
        self.info_dictionary=info_dictionary

    def print_servo(self):
        # Order to be printed:
        # id, stateID, voltage, temperature, angle, speed
        # The dictionary has a boolean value declaring whether the motor is moving.

        #stored to a temporary value for readability
        idict=self.info_dictionary
        if idict['moving']:
            state = 'RUNNING'
        
        else:
            state = 'IDLE'
        
        # Prints the info out in one line as a formatted string.
        # Every element is seperated by a space.
        print('{id} {state} {voltage} {temp} {pos} {speed}'.format(id = self.output_name, 
        state = state, voltage = idict["voltage"], temp = idict["temperature"], 
        pos = idict["position"]*.088, speed = idict["speed"]))


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

        # A list of ServoLogs, indexed by servo names(formatted as above).
        # It is used to speed up gathering information, by
        # keeping the data from unchanging motors.
        self.log_list={}


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

    
    def move_motor(self, console_input, input_length):
        """
        Function member will enable torque, and then change the goal position,
        which then appends the servo to a list of running motors.
        Returns nothing.
        """
        
        #If we don't have a motor and a position, this is an error
        if input_length != 3:
            return True

        #extract the needed name from the input
        servo_name=console_input[1]
        
        #These values hold the same role as in stop_motor.
        pname = servo_name[:-4]
        sid = int(servo_name[-3:])
        
        # Trying to move a motor not attached to the system would be 
        # just as bad as stopping one.
        if self.manager.check_included(pname,sid):

            # Enable torque for motor to have it set goal position.
            self.manager.ports_by_name[pname].proxy.set_torque_enabled(sid, [1])

            raw_angle = int(console_input[2]) / (88/1000)

            # Set goal position of the motor
            # self.manager.ports_by_name[pname].proxy.set_goal_position(sid,int(console_input[2]*(1000/88)))
            self.manager.ports_by_name[pname].proxy.set_goal_position(sid,(int(console_input[2])))

            # We add the motor to a list of currently running motors.
            self.running_motors.append(servo_name)
        return True


    def update_motor(self, servo_name):
        """
        Stores the full information set for one named servo
        into the appropriate Log.
        No output or returns.
        """
        
        # The port name and motor ID, extracted from the full servo name.
        # Used to navigate the architecture to access the proxy/wrapper.
        pname = servo_name[:-4]
        mid = int(servo_name[-3:])

        # We cannot try to access a motor which isn't attached.
        if self.manager.check_included(pname, mid):
        
            # Defines an empty set which will store the information dictionary.
            idict = {}
        
            # Uses the wrapper to fill that dictionary with the needed entries.
            idict = self.manager.ports_by_name[pname].proxy.get_feedback(mid)
        
            # Since the log list is indexed by servo name, we add the information here to the Log
            self.log_list[servo_name].log_servo(idict)

        else:
            print("No such servo")
        

    def scan(self, console_input, input_length):
        """
        Gathers information on every motor attached to the system,
        setting up the Logs. It *must* be run in order for
        future updates to work.
        """

        # Since the RobotManager has the data structures which are needed
        # to access every servo, the data collection is handled by that class.
        # This function returns an array of dictionaries.
        all_info = self.manager.list_all_servo_info()
        
        # We loop through all the individual dictionaries, and create a servo log for each.
        # This will give us a data structure of every servo attached to the computer, and
        # prepare them to be updated only as needed.
        for lmotor in all_info:

            # Creates the Log, giving it its name and index.
            # if the Log already exists, no errors nor bugs will occur.
            # It just overwrites what's already there.
            self.log_list[lmotor['id']]=ServoLog(lmotor['id'])
            
            #The information in the dictionary is passed to the Log.
            self.log_list[lmotor['id']].log_servo(lmotor)

            #The Log prints all of its information.
            self.log_list[lmotor['id']].print_servo()
        
        return True


    def running_update(self, console_input, input_length):
        
        # Uses the Drivers to update *only* the motors
        # which are running, and therefore changing.
        for rmotor in self.running_motors:
            self.update_motor(rmotor)

        #Prints out every motor.
        for servo_log in self.log_list:
            self.log_list[servo_log].print_servo()
        
        return True


    def end_listening(self, console_input, input_length):
        #Just returns false, causing the listener to exit its loop
        # and the program to end.
        return False


    def ignore(self, console_input, input_length):
        #Used to prevent errors in the switch.
        return True


    def listen(self):
        """
        This is the function that MintPatch will be in while idle.
        Some Output is handled here.
        However, the main purpose of this function is to parse input and
        call the appropriate functions to handle the command cases.
        """

        # This dictionary will be used as a switch statement
        # to quickly parse input.
        # It translates strings into the appropriate function,
        # Each of which has the same parameters and return.
        switch_dict={
            'move': self.move_motor,
            'scan': self.scan,
            'update': self.running_update,
            'end' : self.end_listening
        }


        # LOOP SETUP
        Continue=True

        # Establishes a common, potentially infinite while loop.
        # It can be ended by inputing "end".
        while(Continue):
            
            # GATHERING INPUT
           
            # NODE.JS
            # Uses this input function to gather one line from the Node.
            # This is a string. 
            
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

            # In this case, there is no input.
            # There's no reason to go through the loop.
            if l == 0:
                continue


            # SWITCH

            # This is the magical switch statement. It runs the first
            # word of the input through the switch dictionary,
            # finding the appropriate function to run. Then, it
            # passes the full input array and its length. Not every
            # function uses them, but this allows for a very efficient call.

            # In the case that the input is faulty, the switch will call the
            # "ignore" function. This does nothing.

            # Finally, each function returns a Boolean telling the listener
            # whether or not to continue the loop. Every function except "end"
            # returns true.
            Continue=switch_dict.get(cinar[0], self.ignore)(cinar,l)
            

        # END LOOP
        print("Thank you for using MintPatch!")
