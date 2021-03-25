"""
Responsible for communication between the GUI and library.
Currently uses a python consule
Nathan Moder
3/23/2021
"""

class GUITranslator:

    #constructor. mostly empty; translator should be created by the manager which sends itself
    #TODO: Debug
    def __init__(self,_manager):
        self.manager=_manager
        self.running_motors=[]
    
    #an internal functions for readability. prints the info in the dictionaries from the wrapper
    def print_from_dict(self, idict):
        #order to be printed:
        #id, stateID, voltage, temperature, angle, speed

        #use moving bool to give state
        state='OFF'
        if idict['moving']:
            state='RUNNING'
        else:
            state='IDLE'
        
        print('{id} {state} {voltage} {temp} {pos} {speed}'.format(id=idict["id"],state=state,voltage=idict["voltage"],temp=idict["temperature"],pos={idict["position"]},speed=idict["speed"]))    
    
    #a function for updating all running servo motors.
    #TODO: Test once attached to moving servos. Cannot yet be debugged.
    def update_running(self):
        raise NotImplementedError
        for motor in self.running_motors:
            if(self.manager.check_included(motor)):
                motor_info=self.manager.port_name[motor].wrapper.get_feedback(motor)
                #Query reaches all the way through the architecture; through the servo manager, to the port manager,
                #and finally to the wrapper.
                self.print_from_dict(motor_info)
            else:
                break
        #necessary information is stored in the class
    

    #a function for updating the all the information on every servo motor.
    #TODO: Should be ready to go; attempt with real servos.
    def update_all(self):
        
        all_info=self.manager.list_all_servo_info()
        #this is a dictionary array indexed by simple integers
        
        for lmotor in all_info:
            self.print_from_dict(lmotor)   
    
    #this is the function that MintPatch will be in while idle.
    #TODO: Learn about python input streams
    #TODO: Implement the parsing as case statements using function variables.
    def listen(self):
        cont=True
        while(cont):
            conin=input()
            cinar=conin.split()
            #diagnostic for loop
            l=cinar.__len__()
            """
            An if-else chain will NOT be used in the final version.
            It is used here to expediate testing
            """
            if l==0:
                continue #I'm sorry my teachers
            if cinar[0]=="update":
               if l==1: 
                   continue
            if cinar[0]=="list":
                for string in self.manager.list_servos():
                    print(string)
                continue

            if cinar[0]=="scan":
                self.update_all()
                continue
            if cinar[0]=="end":
                cont=False

            #TODO: Learn python switch theory
            #TODO: Learn lambda and function objects for python
            """
            cases:
            end: "disconnect"
            scan: "add_scan"
            running: "update_running"
            all: "update_all"
            update <servoID>: "update_single_servo"
            speed <servoID>: "
            set <attribute> <servoID>
            run <servoID>
            stop <servoID>
            """
            #this is the function that MintPatch will be in while idle.
            #we need to establish a communication protocol for
            #the python command line.