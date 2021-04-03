"""
Responsible for communication between the GUI and library.
Currently reads from JSON and outputs to consule.
Legacy version uses a python consule.
Nathan Moder
3/29/2021
"""

#Imports for interaction with Node.js
import sys,json

#Uses funcitons from RobotManager and the wrapper, but does not need to import them.


class GUITranslator:

    #constructor. Just needs a RobotManager called _manager, which the GUITranslator will access
    def __init__(self,_manager):
        
        self.manager=_manager
            #The RobotManager the translator will use.
        
        self.running_motors=[]
            #An initially empty list of the servo names of all running motors
            #A servo name includes both the port and the ID, formatted:
                #<portname>_<motorID>


    #Boiler-plate code for interacting with the commands sent from Node.js
    #Returns a string
    def read_from_json(self):

        jin=sys.stdin.readline()
            #jin becomes the next line in the JSON file created by the Node.
            #It needs to be translated
        print(json.loads(jin))
        return json.loads(jin)
            #loads() does the translation



    #An internal functions for readability. Prints the info out of a Dictionary from the wrapper.
    #Output, but no returns
    def print_from_dict(self, idict):
            #idict is the information dictionary from a single motor.

        #order to be printed:
            #id, stateID, voltage, temperature, angle, speed

        
        if idict['moving']:     #The dictionary has a boolean value declaring whether the motor is moving.
            state='RUNNING'
        else:
            state='IDLE'
        
        print('{id} {state} {voltage} {temp} {pos} {speed}'.format(id=idict["id"],state=state,voltage=idict["voltage"],temp=idict["temperature"],pos={idict["position"]},speed=idict["speed"]))    
            #Prints the info out in one line as a formatted string.
                #Every element is seperated by a space.
        
    
    #Halts the movement of the named servo
    #No returns or output
    def stop_motor(self,servo_name):

        pname=servo_name[:-4]
            #The name of the port is the first part of the string.
                #We use python string operations to remove the last 2 characters.

        sid=int(servo_name[-3:])
            #The ID of the motor is just the last element.


        #these two variables can be adjusted if we allow 2-digit or 3-digit motor ID's


        if self.manager.check_included(pname,sid):
            #attepting to stop a motor that doesn't exist would be bad. many errors.
            
            # TODO: make sure it is fully working
            self.manager.ports_by_name[pname].proxy.set_goal_velocity(sid,0)
                #Uses a function in the wrapper to set the goal velocity to zero. This will slow the motor to a stop.
                    #ports_by_name is indexed by the port name, while the wrapper function needs the motor ID

            self.running_motors.remove(servo_name)
                #removes the name from the internal list. it is no longer known to be running

        
        #This path depends on our safety priorities.
        """
        else:
            #if a servo is entered that doesn't seem to exist, we assume
            #that the typo was dangerous and stop the whole system
            self.stop_all_motors()
        """

    #Halts all movement in the system.
    #No returns or output.
    def stop_all_motors(self):

        for motor in self.running_motors:
            self.stop_motor(motor)
        #We loop though ever motor that we know to be running, and stop them.

    def move_motor(self, servo_name, speed):

        #These values hold the same role as in stop_motor
        pname=servo_name[:-4]
        sid=int(servo_name[-3:])

        if self.manager.check_included(pname,sid):
            #Trying to move a motor not attached to the system would be just as bad as stopping one
        
            self.manager.ports_by_name[pname].proxy.set_goal_velocity(sid,int(speed))
                # WIP

            self.running_motors.append(servo_name)
                #We add the motor to a list of currently running motors.


    #Prints the full information set for one named servo.
    #Output, but no returns.
    def update_motor(self, servo_name):

        #These values hold the same role as in stop_motor
        pname=servo_name[:-4]
        sid=int(servo_name[-3:])
        
        #debug prints
        # print("DEBUG PNAME: " + str(pname))
        # print("DEBUG SID: " + str(sid))

        if self.manager.check_included(pname, sid):
            #We cannot try to access a motor which isn't attached.
        
            idict={}
                #Defines an empty set which will store the information dictionary
        
            idict=self.manager.ports_by_name[pname].proxy.get_feedback(sid)
                #Uses the wrapper to fill that dictionary with the needed entries
        
            idict['id']=servo_name
                #The motor ID in the dictionary does not include the port.
                    #Since we want to output a combined name, we alter the entry

            self.print_from_dict(idict)
                #Since this was a dictionary returned from get_feedback, we can output it with
                #this internal function

        else:
            print("No such servo")
        
    #a function for updating the all the information on every servo motor.
    def update_all(self):
        
        all_info=self.manager.list_all_servo_info()
            #Since the RobotManager has the data structures which are needed
            #to access every servo, the data collection is handled by that class.
                #This function returns an array of dictionaries.
        
        for lmotor in all_info:
            self.print_from_dict(lmotor)   
            #We loop through all the individual dictionaries, and print them out.
    
    """
    This is the function that MintPatch will be in while idle.
    Some Output is handled here.
    However, the main purpose of this function is to parse input and
    call the appropriate functions to handle the command cases.
    """
    def listen(self):
        
        """
        LOOP SETUP
        """
        print("ENTER LOOP")
        Continue=True
        while(Continue):
            #Establishes a common, potentially infinite while loop.
                #It can be ended by inputing "end".

            """
            GATHERING INPUT
            """

            """
            Node.js
            """
            #conin=self.read_from_json()
                #Uses this input function to gather one line from the Node.
                    #This is a string.         
            

            """
            Console
            """
            conin = sys.stdin.readline()
            #or:
            #conin=sys.stdin.readline()

            """
            Input Setup
            """

            cinar=conin.split()
                #Since it's a string, and the parts of the input should be seperated
                #by spaces; we split this to make it an array.
            
            l=cinar.__len__()
                #Knowing the length of this string array will
                #let us catch errors and quickly skip faulty commands


            """
            HANDLE DIFFERENT COMMANDS
            """

            if l==0:
                #In this case, there is no input.
                #There's no reason to go through the loop.
                continue
            

            if cinar[0]=="stop":
                #In this case, we know we will be stopping motors.

                if l==1:
                    #If the word is alone, we assume this is an emergency stop.
                    self.stop_all_motors()

                else:
                    #Otherwise, we let the user specify a single motor to stop.
                    
                    if l!=2:
                        #If they try to specify more than one motor, the format is wrong.
                        continue
                    
                    self.stop_motor(cinar[1])
                    #Correctly called, we stop the motor.
                continue

            if cinar[0]=="move":
                if l!=3:
                    #We need both a servo and a speed, and nothing else.
                    continue
                self.move_motor(cinar[1],cinar[2])
                continue


            if cinar[0]=="update":
                if l==1: 
                   continue
                self.update_motor(cinar[1])
                continue


            if cinar[0]=="running":
                #Updates every running motor.

                for motor in self.running_motors:
                    self.update_motor(motor)
                    #uses the update_motor function on every motor in the array.
                continue
            
            if cinar[0]=="list":
                for string in self.manager.list_servos():
                    print(string)
                continue

            if cinar[0]=="scan":
                self.update_all()
                continue

            if cinar[0]=="end":
                Continue=False

        #END LOOP
        print("Thank you for using MintPatch!")
