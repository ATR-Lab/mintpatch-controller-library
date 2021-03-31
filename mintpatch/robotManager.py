"""
Creates the Servo Manager, which will store information about
which port each servo is attached to.
It also handles the startup of MintPatch.
Nathan Moder
3/29/2021
"""

from portManager import PortManager


class RobotManager:

    """Constructor. _port_list is an array of strings; this constructor will set up every port manager.
    _setup_info is a dictionary of settings that are used for the ports; it is indexed by port name.
    After execution, the manager and all port managers will be ready to function.
    TODO: Implement code to get rid of empty portManagers, if we'll be checking every port of the appropriate type"""
    def __init__(self, _port_list, _setup_info):
        
        """
        Defines empty arrays
        """

        self.servo_list=[] #array of pairs of (port_name(string), servo_id(int))
        self.ports_by_name={} #a dictionary that connects the PortManager objects to strings of their names
        self.port_names=_port_list #string array of the paths to the ports
        

        """
        for every port listed in the arguments, it makes a
        port manager. then, it makes not of every servo attached to it.
        """


        for portn in _port_list:
            #Enter the loop for every port name.
            temp_manager=PortManager(portn,_setup_info[portn])

            if not temp_manager.is_empty():
                #We want to make sure we aren't storing empty ports.

                self.ports_by_name[portn]=temp_manager
                    #For this port, a Port Manager is set up.
                        #Since _setup_info is indexed by port name, we simply index it
                        #The same name will is used to index ports_by_name 
                                   
            else:
                continue
                #If it is empty, we have no need to go into the other loop.

            for servo in self.ports_by_name[portn].servos:
                #Utilizes and looks through the list of servos in the port manager.

                self.servo_list.append((portn, servo))
                    #Stores a pair of values in the servo_list.
                        #We want it to seperately track the port and servo ID for later ease.
        #debug print
        #print(self.servo_list)



    """
    Simple seach through the servo list to see if it exists.
    Could be used for preventing errors with faulty indexes
    and tool calls.
    Returns a boolean.
    """
    def check_included(self, pname, sid):
        #Uses itterate and keep track algorithm
        found=False

        for pair in self.servo_list:
        
            if pair[0]==pname and int(pair[1])==int(sid):
                found=True
            #As this is a pair of port name and servo ID, we check both to see
            #if we found our matching servo.
       
        return found
    
    #returns the string list of every servo's name
    def list_servos(self):
        names=[]
        for pair in self.servo_list:
            psid=pair[1]
            sid=''
            if psid<10:
                sid='00'+str(psid)
            elif psid<100:
                sid='0'+str(psid)
            else:
                sid=str(psid)
            names.append('{pname}_{sid}'.format(pname=pair[0],sid=sid))
        return names

    """
    goes through every servo and gets info
    uses PortManager and Wrapper
    returns an array of dictionaries
    """
    def list_all_servo_info(self):

        info_dict=[]
            #Defines the empty array which will hold the dictionaries
        
        for portn in self.port_names:
            port=self.ports_by_name[portn]
            #Loops through every Port Manager using their name,
            #since that is how they are stored

            for ids in port.servos:
                #This loop goes through the servos on the port. 
                    #ids is an integer number that corresponds
                    #to the value in a register on the motor.

                temp_dict=port.wrapper.get_feedback(ids)
                    #Accesses the wrapper held by the port manager.
                        #From it, it uses get_feedback to obtain a comprehensive
                        #list of dynamic information from the servo.
                            #TODO: Add somewhere a means of obtaining static info as well.
                sid=''
                if ids<10:
                    sid='00'+str(ids)
                elif ids<100:
                    sid='0'+str(ids)
                else:
                    sid=str(ids)

                temp_dict['id']='{portn}_{id}'.format(portn=portn,id=sid)
                    #The ID returned only describes the motor, but we need to
                    #provide the port name for it to be useful to outside systems.
                        #We use the format function to create our new Servo Name:
                            #<port_name>_<motor_id>
                            #For example, for motor 5 on the port "USB0", we would have:
                                #USB0_5
                        #TODO: Must currently be adapted if we want double-digit motor ID's

                info_dict.append(temp_dict)
                    #We add the obtained and edited dictionary to the list we will return.

        return info_dict

    #Returns an array of the names of every port.
    def list_ports(self):
        return self.port_names
