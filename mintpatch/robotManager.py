"""
Creates the Servo Manager, which will store information about
which port each servo is attached to.
It also handles the startup of MintPatch.
Nathan Moder
3/23/2021
"""

from portManager import PortManager


class RobotManager:

    """Constructor. _port_list is an array of strings; this constructor will set up every port manager.
    _setup_info is a dictionary of settings that are used for the ports; it is indexed by port name.
    After execution, the manager and all port managers will be ready to function.
    TODO: Implement code to get rid of empty portManagers, if we'll be checking every port of the appropriate type"""
    def __init__(self, _port_list, _setup_info):
        #defines empty arrays
        self.servo_list=[] #array of pairs of (port_name(string), servo_id(int))
        self.ports_by_name={} #a dictionary that connects the PortManager objects to strings of their names
        self.port_names=_port_list #string array
        
        #TODO: temp list may be outdated. revise
        temp_port_list={}
        

        """
        for every port listed in the arguments, it makes a
        port manager. then, it makes not of every servo attached to it.
        """

        #TODO: variables outside of loops is outdated.
        # update and make more condusive to python norms
        i=0
        j=0
        for item in _port_list:
           
            #debug print
            #print(item) 
           
            a=PortManager(item,_setup_info[item])
            temp_port_list[i]=a
            self.ports_by_name[a.port_name]=a

            for servo in temp_port_list[i].servos:
            
                self.servo_list.append((temp_port_list[i].port_name, servo))
                j=j+1
            
            i=i+1
        

    """
    simple seach through the servo list to see if it exists.
    could be used for map security
    TODO: Class structure has changed since this itteration. update
    """
    def check_included(self, _query_servo):
        in_manager=False    
        for servo in self.servo_list:
            if servo==_query_servo:
                in_manager=True
        return in_manager
    
    #returns the string list of every servo's name
    def list_servos(self):
        names=[]
        for pair in self.servo_list:
            names.append('{pname}_{sid}'.format(pname=pair[0],sid=pair[1]))
        return names

    """
    goes through every servo and gets info
    uses PortManager and Wrapper
    TODO: Debug for real servos
    returns an array of dictionaries
    """
    def list_all_servo_info(self):

        info_dict=[]
        
        for portn in self.port_names:
            #stores the PortManager in port
            port=self.ports_by_name[portn]
        
            for ids in range(0,port.servos.__len__()):
                
                temp_dict=port.wrapper.get_feedback(port.servos[ids])
                temp_dict['id']='{portn}_{id}'.format(portn=portn,id=port.servos[ids])
                info_dict.append(temp_dict)

        return info_dict

    #Returns an array of the names of every port.
    #Does not return repeats.
    #TODO: Debug
    def list_ports(self):
        return self.port_names
