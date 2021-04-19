# Standard Library imports
import math
import sys
import errno
import rospy

# Thread / Locking imports
from collections import deque
from threading import Thread
from collections import defaultdict

# Dynamixel Messages import for JointState, DynamixelState, MotorStateList, and MotorState
"""
from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import JointState as DynamixelState
from dynamixel_msgs.msg import MotorStateList
from dynamixel_msgs.msg import MotorState
"""

# Port and Packet Handler imports
import dynamixel_sdk.port_handler as port_h
import dynamixel_sdk.packet_handler as packet_h

# Custom Driver imports
from dynomix_driver import sdk_serial_wrapper
from dynomix_tools.dynamixel_control_table import MODEL_NUMBER_2_MOTOR_NAME
from dynomix_tools import dynamixel_tools
from dynomix_driver import sdk_serial_wrapper
from dynomix_driver.dyn_const import *

# Leave here for debugging
  # import roslib
  # roslib.load_manifest('dynamixel_drive')

# TODO: End goal is to have two functions, one called rawPos2Degree and one called
# Degree2RawPos. The function shall be called in the proxy, and then called into the
# Dynamixel for further translations. If the motor uses pulses for determining
# Angles (e.g. fetch from dynamixel_const.py MODELS_TO_PARAM dictionary) or if they
# use the constant of 1 unit = 0.088 degrees. If it's the latter, return either the
# raw position (degree / 0.088) or the degree (raw_pos * 0.088); but if it's puleses
# then go fetch the value from the table and return either the raw position
# (degree / pulse_unit) or the degree (raw_pos * pulse_unit). And pulse units are
# typically 180/max_position_limit (though it might be a fraction off, such as
# with the H54 having the formula as 'raw_pos * 180/250961.5', so we'll have the
# pulse units already calculated and listed in the model_to_param table).
# Function List:
# - input: raw_pos, servo_id, model_info (note: will only be needed in the proxy, 
# only raw_pos is required for the call)
# - model_name <- model_info[servo_id]
# - if model_to_param[model_name][pulse] fetches nothing, return needed calculation above.
# - if pulse unit is found, then return needed calculations above with pulse variable.
# - Output: either translated degree or translated raw position


class DynomixSerialProxy():
  """
  Serial proxy that initializes dynamixel_sdk  port handler and packet handler
  """
  def __init__(self, 
    port_name='/dev/ttyUSB0', 
    port_namespace='ttyUSB0', 
    baud_rate=1000000, 
    min_motor_id=1,
    max_motor_id=25,
    update_rate=5,
    diagnostics_rate=1,
    error_level_temp=75,
    warn_level_temp=70,
    readback_echo=False,
    protocol_version=2.0):
  
    # Initialize variables for Serial Proxy
    self.port_name          = port_name
    self.port_namespace     = port_namespace
    self.baud_rate          = baud_rate
    self.min_motor_id       = min_motor_id
    self.max_motor_id       = max_motor_id
    self.update_rate        = update_rate
    self.diagnostics_rate   = diagnostics_rate
    self.error_level_temp   = error_level_temp
    self.warn_level_temp    = warn_level_temp
    self.readback_echp      = readback_echo
    self.protocol_version   = protocol_version

    self.actual_rate        = update_rate 
    self.error_counts       = {'non_fatal': 0, 'checksum': 0, 'dropped': 0}
    # self.current_state      = MotorStateList()
    self.num_ping_retries   = 5
    self.dynotools          = dynamixel_tools.DynamixelTools()
    self.sdk_io             = sdk_serial_wrapper.SDKSerialWrapper(port_name, baud_rate)
    self.angles             = {}

    # Start to publish motor states
    # self.motor_states_pub = rospy.Publisher('motor_states/%s' % self.port_namespace, MotorStateList, queue_size = 1)


  def connect(self):
    """
    Connects up physical port with the port handler, and initialize packet handler
    """
    try:
      # Port and packet handler set up
      self.port_handler   = port_h.PortHandler(self.port_name)
      self.packet_handler = packet_h.PacketHandler(self.protocol_version)

      # Set up port and baud rate
      self.port_handler.openPort()
      self.port_handler.setBaudRate(self.baud_rate)
      self.__find_motors()
    except rospy.ROSInterruptException: pass

    self.running = True


  def disconnect(self):
    """
    Disconnects motors
    """
    self.running = False


  def __fill_motor_parameters(self, motor_id, model_number):
    """
    Stores some extra information about each motor on the parameter server.
    Some of these paramters are used in joint controller implementation.
    """
    # Get the Motor Name
    model_name = self.dynotools.getModelNameByModelNumber(model_number)

    # Get Max and Min angles
    angles = self.sdk_io.get_angle_limits(motor_id, model_name)
    self.angles[str(model_name)] = angles 

    # Get Current, Min, and Max voltages
    voltage = self.sdk_io.get_voltage(motor_id, model_name)
    voltages = self.sdk_io.get_voltage_limits(motor_id, model_name)
    
    # ROS Parameters Setup
    """
    rospy.set_param('dynamixel/%s/%d/model_number' %(self.port_namespace, motor_id), model_number)
    rospy.set_param('dynamixel/%s/%d/model_name' %(self.port_namespace, motor_id), DXL_MODEL_TO_PARAMS[model_number]['name'])
    rospy.set_param('dynamixel/%s/%d/min_angle' %(self.port_namespace, motor_id), angles['min'])
    rospy.set_param('dynamixel/%s/%d/max_angle' %(self.port_namespace, motor_id), angles['max'])
    torque_per_volt = DXL_MODEL_TO_PARAMS[model_number]['torque_per_volt']
    rospy.set_param('dynamixel/%s/%d/torque_per_volt' %(self.port_namespace, motor_id), torque_per_volt)
    rospy.set_param('dynamixel/%s/%d/max_torque' %(self.port_namespace, motor_id), torque_per_volt * voltage)
      
    velocity_per_volt = DXL_MODEL_TO_PARAMS[model_number]['velocity_per_volt']
    rpm_per_tick = DXL_MODEL_TO_PARAMS[model_number]['rpm_per_tick']
    rospy.set_param('dynamixel/%s/%d/velocity_per_volt' %(self.port_namespace, motor_id), velocity_per_volt)
    rospy.set_param('dynamixel/%s/%d/max_velocity' %(self.port_namespace, motor_id), velocity_per_volt * voltage)
    rospy.set_param('dynamixel/%s/%d/radians_second_per_encoder_tick' %(self.port_namespace, motor_id), rpm_per_tick * RPM_TO_RADSEC)
      
    encoder_resolution = DXL_MODEL_TO_PARAMS[model_number]['encoder_resolution']
    range_degrees = DXL_MODEL_TO_PARAMS[model_number]['range_degrees']
    range_radians = math.radians(range_degrees)
    rospy.set_param('dynamixel/%s/%d/encoder_resolution' %(self.port_namespace, motor_id), encoder_resolution)
    rospy.set_param('dynamixel/%s/%d/range_degrees' %(self.port_namespace, motor_id), range_degrees)
    rospy.set_param('dynamixel/%s/%d/range_radians' %(self.port_namespace, motor_id), range_radians)
    rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_degree' %(self.port_namespace, motor_id), encoder_resolution / range_degrees)
    rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_radian' %(self.port_namespace, motor_id), encoder_resolution / range_radians)
    rospy.set_param('dynamixel/%s/%d/degrees_per_encoder_tick' %(self.port_namespace, motor_id), range_degrees / encoder_resolution)
    rospy.set_param('dynamixel/%s/%d/radians_per_encoder_tick' %(self.port_namespace, motor_id), range_radians / encoder_resolution)
    """

    # keep some parameters around for diagnostics
    self.motor_static_info[motor_id] = {}
    self.motor_static_info[motor_id]['model'] = DXL_MODEL_TO_PARAMS[model_number]['name']
    self.motor_static_info[motor_id]['min_angle'] = angles['min']
    self.motor_static_info[motor_id]['max_angle'] = angles['max']


  def __find_motors(self):
    """
    Function to add motors into motor container with servo id, model name and model number
    """
    # Start with Motor Info declaration
    # rospy.loginfo('%s: Pinging motor IDs %d through %d...' % (self.port_namespace, self.min_motor_id, self.max_motor_id))
    print('%s: Pinging motor IDs %d through %d...' % (self.port_namespace, self.min_motor_id, self.max_motor_id))
    self.motors = []
    self.motor_static_info = {}
    self.motor_info = {}

    # Getting Motor IDs
    for motor_id in range(self.min_motor_id, self.max_motor_id + 1):
      for trial in range(self.num_ping_retries):
        try:
          result = self.packet_handler.ping(self.port_handler, motor_id)

        except Exception as ex:
          rospy.logerr('Exception thrown while pinging motor %d - %s' % (motor_id, ex))
          continue

        if not result[1]: # If no error was returned, add motor ID
          self.motors.append(motor_id)
          break

    # Failure to find any motors :-(
    if not self.motors:
      rospy.logfatal('%s: No motors found.' % self.port_namespace)
      sys.exit(1)
            
    counts = defaultdict(int)

    to_delete_if_error = []
    # rospy.loginfo("Getting motor numbers.......")
    print("Getting motor numbers.......")

    # Find the model numbers asscoiated with the Motor IDs
    for motor_id in self.motors:
      for trial in range(self.num_ping_retries):
        model_number = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, 0)
        # rospy.logwarn("MOTOR_ID: " + str(motor_id))
        # rospy.logwarn("MODEL_NUMBER: " + str(model_number[0]))
        # rospy.logwarn("ERROR: " + str(model_number[1]))
        print("MOTOR_ID: " + str(motor_id))
        print("MODEL_NUMBER: " + str(model_number[0]))
        print("ERROR_NUMBER: " + str(model_number[1]))

        # Fill Motor Parameters
        self.__fill_motor_parameters(motor_id, model_number[0])
        counts[model_number[0]] += 1
        self.motor_info[str(motor_id)] = {"model_number": model_number[0],
                                          "max_angle": self.motor_static_info[motor_id]['max_angle'],
                                          "min_angle": self.motor_static_info[motor_id]['min_angle']} # IRVIN

        break
    
    # If any errors happen, remove the motor from the list
    for motor_id in to_delete_if_error:
      self.motors.remove(motor_id)

    # rospy.set_param('dynamixel/%s/connected_ids' % self.port_namespace, self.motors)

    # rospy.set_param('dynamixel/%s/motor_info' % self.port_namespace, self.motor_info) # IRVIN

    status_str = '%s: Found %d motors - ' % (self.port_namespace, len(self.motors))

    # Get status of each motor
    for model_number,count in counts.items():
      if count:
        if model_number in MODEL_NUMBER_2_MOTOR_NAME:
          model_name = MODEL_NUMBER_2_MOTOR_NAME[model_number]['name']
          status_str += '%d %s [' % (count, model_name)
            
          for motor_id in self.motors:
            if motor_id in self.motor_static_info and self.motor_static_info[motor_id]['model'] == model_name:
              status_str += '%d, ' % motor_id  
          
          status_str = status_str[:-2] + '], '

    # rospy.loginfo('%s, initialization complete.' % status_str[:-2])
    print('%s, initialization complete.' % status_str[:-2])


  # For ROS Implementation
  """
  def __update_motor_states(self):
    num_events = 50
    rates = deque([float(self.update_rate)]*num_events, maxlen=num_events)
    last_time = rospy.Time.now()
    
    rate = rospy.Rate(self.update_rate)
    while not rospy.is_shutdown() and self.running:
      # get current state of all motors and publish to motor_states topic
      motor_states = []
      for motor_id in self.motors:
        try:
          state = self.get_feedback(motor_id)
          if state:
            motor_states.append(MotorState(**state))
        except OSError, ose:
          if ose.errno != errno.EAGAIN:
            rospy.logfatal(errno.errorcode[ose.errno])
            rospy.signal_shutdown(errno.errorcode[ose.errno])
            
      if motor_states:
        msl = MotorStateList()
        msl.motor_states = motor_states
        self.motor_states_pub.publish(msl)
        
        self.current_state = msl
        
        # calculate actual update rate
        current_time = rospy.Time.now()
        rates.append(1.0 / (current_time - last_time).to_sec())
        self.actual_rate = round(sum(rates)/num_events, 2)
        last_time = current_time
          
      rate.sleep()
  """

  # TODO: Get these working without calling for serial proxy
  def set_goal_position(self, servo_id, goal_position):
    return self.sdk_io.set_goal_position(servo_id, goal_position, self.motor_info)

  def set_torque_enabled(self, servo_id, enabled):
    return self.sdk_io.set_torque_enabled(servo_id, self.motor_info, enabled)

  def get_feedback(self, servo_id):
    return self.sdk_io.get_feedback(servo_id, self.motor_info)


if __name__ == '__main__':
  try:
    serial_proxy = DynomixSerialProxy()
    serial_proxy.connect()
    serial_proxy.disconnect()
  except rospy.ROSInterruptException: pass