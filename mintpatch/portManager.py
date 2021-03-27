"""
Holds all the Serial Proxies and the singular SDK Wrapper assigned
assigned to a single Port.
Nathan Moder
3/23/2021
"""

# Emulated Motors
# from fakeWrapper import SDKSerialWrapper
# from fakeProxy import DynomixSerialProxy

# Real Motors
from dynomix_driver.sdk_serial_wrapper import SDKSerialWrapper
from dynomix_driver.dynomix_serial_proxy import DynomixSerialProxy

class PortManager:
    """
    constructor. takes a string of the port name as input, and uses it to set up ROS connections.
    also needs the settings that will be used to set up the proxies and wrapper.
    on completion, the PortManager, SerialProxy, and Wrapper will function.
    """
    def __init__(self,_port_name, _setup_info):
	rospy.init_node('portManager', anonymous=True)
        self.port_name=_port_name
        self.wrapper=SDKSerialWrapper('/dev/{_port_name}'.format(_port_name=_port_name),_setup_info["baudrate"])
        self.proxy=DynomixSerialProxy("/dev/{_port_name}".format(_port_name=_port_name),_port_name, _setup_info["baudrate"],_setup_info["minID"],_setup_info["maxID"],_setup_info["updateRate"],_setup_info["diagnosticsRate"])
        self.proxy.connect()
        self.servos=[]

        #debug print
        #print(f'port {_port_name} has servos {self.proxy.motors}')
        
        for i in range(0,self.proxy.motors.__len__()):
            self.servos.append(self.proxy.motors[i])
            self.wrapper.get_feedback(self.servos[i])
        
    
    #returns whether or not any servos are attached to the port.
    #TODO: Implement
    def is_empty(self):
        return False
