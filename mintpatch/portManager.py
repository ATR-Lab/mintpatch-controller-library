"""
Holds the Serial Proxies and the SDK Wrapper assigned
assigned to a single Port.
Nathan Moder & M. Arnett
3/28/2021
"""

# Imports for Emulated Motors
from fakeWrapper import SDKSerialWrapper
from fakeProxy import DynomixSerialProxy

# Imports for Real Motors
#from dynomix_driver.sdk_serial_wrapper import SDKSerialWrapper
#from dynomix_driver.dynomix_serial_proxy import DynomixSerialProxy
#import rospy

class PortManager:
    """
    Constructor. Takes a string of the port name as input, and uses it to set up ROS connections.
    Also needs the settings that will be used to set up the proxies and wrapper.
    On completion, the PortManager, SerialProxy, and Wrapper will function.
    """
    def __init__(self,_port_name, _setup_info):
        
        self.port_name = _port_name
            #The port_name refers to the path to the USB port the motors are attached to

        #rospy.init_node('portManager', anonymous=True)  #Initializes a ROS node for this port.
            #Necessary for the Wrapper and Proxy to work.
        
        self.wrapper=SDKSerialWrapper('/dev/{_port_name}'.format(_port_name=_port_name),_setup_info["baudrate"])
            #Constructs a wrapper for the port using the settings provided

        self.proxy=DynomixSerialProxy("/dev/{_port_name}".format(_port_name=_port_name),_port_name, _setup_info["baudrate"],_setup_info["minID"],_setup_info["maxID"],_setup_info["updateRate"],_setup_info["diagnosticsRate"])
            #Constructs a proxy for the port using the settings provided
        
        self.proxy.connect()        #Activates the proxy
            #Turns on the connection to the physical motors.
            #Gathers static information about them.
            #Stores the list of motor id's attached in proxy.motors

        self.servos=self.proxy.motors   #Stores the list of motors in the PortManager
            #This may not be strictly necessary
            #TODO: Consider removing and calling down to the proxy for needs.

        #debug print
        #print('port {_port_name} has servos {motors}'.format(_port_name=_port_name,motors=self.proxy.motors))
    
    #returns whether or not any servos are attached to the port.
    def is_empty(self):
        return self.servos.__len__==0
