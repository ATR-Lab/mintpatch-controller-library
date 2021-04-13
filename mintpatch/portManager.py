"""
Holds the Serial Proxies and the SDK Wrapper assigned
assigned to a single Port.
Original Author: Nathan Moder 
Cleaned up by: M. Arnett
3/28/2021
"""


# Imports for Emulated Motors
# from fakeWrapper import SDKSerialWrapper
# from fakeProxy import DynomixSerialProxy


# Imports for Real Motors
from dynomix_driver.sdk_serial_wrapper import SDKSerialWrapper
from dynomix_driver.dynomix_serial_proxy import DynomixSerialProxy
import rospy


class PortManager:
    def __init__(self,_port_name, _setup_info):
        """
        Constructor. Takes a string of the port name as input, 
        and uses it to set up ROS connections.
        Also needs the settings that will be used to set up the proxies and wrapper.
        On completion, the PortManager, SerialProxy, and Wrapper will function.
        """
        # The port_name refers to the path to the USB port the motors are attached to
        self.port_name = _port_name

        # Initializes a ROS node for this port.
        # Necessary for the real Wrapper and Proxy to work.
        rospy.init_node('portManager', anonymous=True)
        
        # Constructs a wrapper for the port using the settings provided
        self.wrapper = SDKSerialWrapper('/dev/{_port_name}'.format(_port_name=_port_name),
                                                                _setup_info["baudrate"])

        # Constructs a proxy for the port using the settings provided
        self.proxy = DynomixSerialProxy("/dev/{_port_name}".format(_port_name=_port_name),
        _port_name,  _setup_info["baudrate"], _setup_info["minID"], _setup_info["maxID"], 
        _setup_info["updateRate"], _setup_info["diagnosticsRate"])
        
        # Activates the proxy
        # Turns on the connection to the physical motors.
        # Gathers static information about them.
        # Stores the list of motor id's attached in proxy.motors
        self.proxy.connect()

        # Stores the list of motors in the PortManager
        # This may not be strictly necessary
        # TODO: Consider removing and calling down to the proxy for needs.
        self.servos = self.proxy.motors

    
    def is_empty(self):
        """
        Returns whether or not any servos are attached to the port.
        """
        return self.servos.__len__ == 0
