# Import standard and threading libraries
import time
import serial
from array import array
from binascii import b2a_hex
from threading import Lock
import rospy

# Import custom libraries and Port / Packet Handler
import dynamixel_sdk.port_handler as port_h
import dynamixel_sdk.packet_handler as packet_h
from dynamixel_sdk import *
from dynamixel_const import *
from dynomix_tools import dynamixel_tools


class SDKSerialWrapper:
  """
  Replacement for Dynamixel-IO, slimed down to only have neccessary functions
  """
  def __init__(self, port, baudrate, feedback_echo=False, protocol_version=2.0):
    # Initialize Port and Packet Handler, also make needed variables / objects
    # TODO: Should try/catch in case of errors
    self.serial_mutex   = Lock()
    self.port_handler   = port_h.PortHandler(port)
    self.packet_handler = packet_h.PacketHandler(protocol_version)
    self.port_handler.openPort()
    self.port_handler.setBaudRate(baudrate)
    self.port = port
    self.baudrate = baudrate
    self.dynotools = dynamixel_tools.DynamixelTools()


  # Delete this? I still need to figure out if this is needed or not
  # def __read_response(self, servo_id):
  #   data = []

  #   try:
  #     data.extend(self.ser.read(4))
  #     if not data[0:2] == ['\xff', '\xff']: raise Exception('Wrong packet prefix %s' % data[0:2])
  #     data.extend(self.ser.read(ord(data[3])))
  #     data = array('B', ''.join(data)).tolist() # [int(b2a_hex(byte), 16) for byte in data]
  #   except Exception, e:
  #     raise DroppedPacketError('Invalid response received from motor %d. %s' % (servo_id, e))

  #   # verify checksum
  #   checksum = 255 - sum(data[2:-1]) % 256
  #   if not checksum == data[-1]: raise ChecksumError(servo_id, data, checksum)

  #   return data


  # TODO: IRVIN NEEDS TO REALLY IMPLEMEMT THIS (I think I might have got it? Still need
  #       to flesh out some stuff. - Marcus)
  def read(self, servo_id, address, size):
    """Read "size" bytes of data from servo with a given "servo_id" at
    the register with "address". "address" is an integer between 0 and 57.
    It is recommended to use the constant in module for readability

    e.g: to read from servo with 1,
      read(1, MX_106_GOAL_POSITION, 2)
    """
    # Reads from Dynamixel SDK packet handler
    with self.serial_mutex:
      result = self.packet_handler.readTxRx(self.port_handler, servo_id, address, size)

      # wait for response packet from the motor
      timestamp = time.time()
      time.sleep(0.0013) #0.00235 is a debug time value

      # read response
      # data = self.packet_handler.getTxRxResult(response)
      # data.append(timestamp)

    return result


  # Delete this? I still need to figure out if this is needed or not
  # def __read_response(self, servo_id):
  #   data = []

  #   try:
  #     data.extend(self.ser.read(4))
  #     if not data[0:2] == ['\xff', '\xff']: raise Exception('Wrong packet prefix %s' % data[0:2])
  #     data.extend(self.ser.read(ord(data[3])))
  #     data = array('B', ''.join(data)).tolist() # [int(b2a_hex(byte), 16) for byte in data]
  #   except Exception, e:
  #     raise DroppedPacketError('Invalid response received from motor %d. %s' % (servo_id, e))

  #   # verify checksum
  #   checksum = 255 - sum(data[2:-1]) % 256
  #   if not checksum == data[-1]: raise ChecksumError(servo_id, data, checksum)

  #   return data

  # TODO: Need to test and verify that this works, somehow.
  def write(self, servo_id, address, data):
    """ Write the values from the "data" list to the servo with "servo_id"
    starting with data[0] at "address", continuing through data[n-1] at
    "address" + (n-1), where n = len(data). "address" is an integer between
    0 and 49. It is recommended to use the constants in module dynamixel_const
    for readability. "data" is a list/tuple of integers.
    To set servo with id 1 to position 276, the method should be called
    like:
        write(1, DXL_GOAL_POSITION_L, (20, 1))
    """
    # Get size variable
    size = len(data)

    print("ENTER WRITE()")
    with self.serial_mutex:
      result = self.packet_handler.writeTxRx(self.port_handler, servo_id, address, size, data)

      print("DEBUG RESULT: " + str(result))

      # wait for response packet from the motor
      timestamp = time.time()
      time.sleep(0.0013) # 0.00235 is a debug time value

    return data

    # Code above here for reference
    # Number of bytes following standard header (0xFF, 0xFF, id, length)
    # length = 3 + len(data)  # instruction, address, len(data), checksum

    # # directly from AX-12 manual:
    # # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
    # # If the calculated value is > 255, the lower byte is the check sum.
    # checksum = 255 - ((servo_id + length + DXL_WRITE_DATA + address + sum(data)) % 256)

    # # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
    # packet = [0xFF, 0xFF, servo_id, length, DXL_WRITE_DATA, address]
    # packet.extend(data)
    # packet.append(checksum)

    # packetStr = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])

    # with self.serial_mutex:
    #   self.__write_serial(packetStr)

    #   # wait for response packet from the motor
    #   timestamp = time.time()
    #   time.sleep(0.0013)

    #   # read response
    #   data = self.__read_response(servo_id)
    #   data.append(timestamp)

    #   return data


  # TODO: Look into this function and figure out how it is used.
  def sync_write(self, address, data):
    """ Use Broadcast message to send multiple servos instructions at the
    same time. No "status packet" will be returned from any servos.
    "address" is an integer between 0 and 49. It is recommended to use the
    constants in module dynamixel_const for readability. "data" is a tuple of
    tuples. Each tuple in "data" must contain the servo id followed by the
    data that should be written from the starting address. The amount of
    data can be as long as needed.
    To set servo with id 1 to position 276 and servo with id 2 to position
    550, the method should be called like:
        sync_write(DXL_GOAL_POSITION_L, ( (1, 20, 1), (2 ,38, 2) ))
    """
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

    with self.serial_mutex:
      self.__write_serial(packetStr)


  def ping(self, servo_id):
    """ Ping the servo with "servo_id". This causes the servo to return a
    "status packet". This can tell us if the servo is attached and powered,
    and if so, if there are any errors.
    """
    with self.serial_mutex:
      result = self.packet_handler(self.port_handler, servo_id)

      # wait for response packet from the motor
      timestamp = time.time()
      time.sleep(0.0013)

      # # read response
      # try:
      #   response = self.__read_response(servo_id)
      #   response.append(timestamp)
      # except Exception, e:
      #   response = []
      # if error:
        # rospy.logerror("Servo ID: %d, 'PING', %s", servo_id, response)
    
    return result


  ##########################################
  # SINGLE SERVO FUNCTIONS
  ##########################################
  def set_torque_enabled(self, servo_id, enabled):
    """
    Sets the value of the torque enabled register to 1 or 0. When the
    torque is disabled the servo can be moved manually while the motor is
    still powered.
    """
    response = self.write(servo_id, DXL_TORQUE_ENABLE, [enabled])
    if response:
      self.exception_on_error(response[4], servo_id, '%sabling torque' % 'en' if enabled else 'dis')
    return response

  # TODO: Change to set_goal_position
  def set_goal_velocity(self, servo_id, goal_position, motor_info):
    """
    Set the servo with servo_id to the specified goal position.
    Position value must be positive.

    loVal = int(position % 256)
    hiVal = int(position >> 8)

    response = self.write(servo_id, DXL_GOAL_POSITION_L, (loVal, hiVal))
    # if response:
        # self.exception_on_error(response[4], servo_id, 'setting goal position to %d' % position)
    return response
    """
    loVal = int(goal_position % 256)
    hiVal = int(goal_position >> 8)

    # Get Model Name and Model Number (e.g., MX_64_T_2 - 311)
    model_number = motor_info[str(servo_id)]['model_number']
    model_name = self.dynotools.getModelNameByModelNumber(model_number)

    # TODO: get the register value for certain indexes of the goal position register
    register_goal_position = self.dynotools.getRegisterAddressByModel(model_name, "goal_position")

    response = self.write(servo_id, register_goal_position, (loVal, hiVal))

    return response




  #################################
  # Servo status access functions #
  #################################
  def get_model_number(self, servo_id):
    """ Reads the servo's model number (e.g. 12 for AX-12+). """
    response = self.read(servo_id, DXL_MODEL_NUMBER_L, 2)
    if response:
      self.exception_on_error(response[4], servo_id, 'fetching model number')
    return response[5] + (response[6] << 8)

  def get_firmware_version(self, servo_id):
    """ Reads the servo's firmware version. """
    response = self.read(servo_id, DXL_VERSION, 1)
    if response:
      self.exception_on_error(response[4], servo_id, 'fetching firmware version')
    return response[5]

  def get_return_delay_time(self, servo_id):
    """ Reads the servo's return delay time. """
    response = self.read(servo_id, DXL_RETURN_DELAY_TIME, 1)
    if response:
      self.exception_on_error(response[4], servo_id, 'fetching return delay time')
    return response[5]

  def get_angle_limits(self, servo_id, model_name):
    """
    Returns the min and max angle limits from the specified servo.
    """
    # read in 4 consecutive bytes starting with low value of clockwise angle limit
    # Register Address and Length variables for Angle Min
    # rospy.logwarn("DEBUG MODEL NAME FOR ANGLE MIN: " + str(model_name))

    register_angle_min = self.dynotools.getRegisterAddressByModel(model_name, "angle_limit_min")
    register_angle_min_length = self.dynotools.getAddressSizeByModel(model_name, "angle_limit_min")
    
    # Register Address and Length variables for Angle Max
    register_angle_max = self.dynotools.getRegisterAddressByModel(model_name, "angle_limit_max")
    register_angle_max_length = self.dynotools.getAddressSizeByModel(model_name, "angle_limit_max")

    # Read using angle min
    raw_response_min = self.read(servo_id, register_angle_min, register_angle_min_length)
    response_min = raw_response_min[0]

    # Read using angle max
    raw_response_max = self.read(servo_id, register_angle_max, register_angle_max_length)
    response_max = raw_response_max[0]

    # rospy.logwarn("DEBUG ANGLE MAX: " + str(response_max))
    # rospy.logwarn("DEBUG ANGLE MIN: " + str(response_min))

    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching CW/CCW angle limits')
      
    # Will crash, still testing above
    # extract data valus from the raw data
    # cwLimit = response[5] + (response[6] << 8)
    # ccwLimit = response[7] + (response[8] << 8)

    # return the data in a dictionary
    # TODO: return right values
    return {'min':response_min[0], 'max':response_max[0]}

  def get_drive_mode(self, servo_id):
    """ Reads the servo's drive mode. """
    response = self.read(servo_id, DXL_DRIVE_MODE, 1)
    if response:
      self.exception_on_error(response[4], servo_id, 'fetching drive mode')
    return response[5]

  def get_voltage_limits(self, servo_id):
    """
    Returns the min and max voltage limits from the specified servo.
    """
    response = self.read(servo_id, DXL_DOWN_LIMIT_VOLTAGE, 2)
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching voltage limits')
    # extract data valus from the raw data
    # rospy.logwarn("DEBUG RESPONSE: " + str(response[0]))
    min_voltage = response[5] / 10.0
    max_voltage = response[6] / 10.0

    # return the data in a dictionary
    return {'min':min_voltage, 'max':max_voltage}

  def get_goal(self, servo_id, model_name):
    """ Reads the servo's goal value from its registers. """
    # Register Address and Length variables
    register_present_goal = self.dynotools.getRegisterAddressByModel(model_name, "goal_position")
    register_present_goal_length = self.dynotools.getAddressSizeByModel(model_name, "goal_position")
    
    # Read using present position
    raw_response = self.read(servo_id, register_present_goal, register_present_goal_length)
    response = raw_response[0]

    # rospy.logwarn("DEBUG GOAL ID: " + str(servo_id))
    # rospy.logwarn("DEBUG GOAL : " + str(response))

    # TODO: Either implement or remove error handling
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching present position')

    # Caculate by bit shift left first index by eight, then add result to index 0
    # Example with [66, 6, 0, 0]: 66 << 8 = 1536; 1536 + 66 = 1602
    goal = response[0] + (response[1] << 8)

    return self.dynotools.convertRawPosition2Degree(goal)

  def get_position(self, servo_id, model_name):
    """ Reads the servo's position value from its registers. """
    # Register Address and Length variables
    register_present_position = self.dynotools.getRegisterAddressByModel(model_name, "present_position")
    register_present_position_length = self.dynotools.getAddressSizeByModel(model_name, "present_position")
    
    # Read using present position
    raw_response = self.read(servo_id, register_present_position - 2, register_present_position_length)
    response = raw_response[0]

    # rospy.logwarn("DEBUG POSITION ID: " + str(servo_id))
    # rospy.logwarn("DEBUG POSITION : " + str(response))

    # TODO: Either implement or remove error handling
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching present position')
    
    # Caculate by bit shift left first index by eight, then add result to index 0
    # Example with [66, 6, 0, 0]: 66 << 8 = 1536; 1536 + 66 = 1602
    position = response[0] + (response[1] << 8)
    return self.dynotools.convertRawPosition2Degree(position)

  def get_speed(self, servo_id, model_name):
    """ Reads the servo's speed value from its registers. """
    # Register Address and Length variables
    register_present_speed = self.dynotools.getRegisterAddressByModel(model_name, "present_velocity")
    register_present_speed_length = self.dynotools.getAddressSizeByModel(model_name, "present_velocity")

    # Read using present position
    raw_response = self.read(servo_id, register_present_speed, register_present_speed_length)
    response = raw_response[0]

    # TODO: Either implement or remove error handling
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching present speed')

    # Caculate by bit shift left first index by eight, then add result to index 0
    # Example with [66, 6, 0, 0]: 66 << 8 = 1536; 1536 + 66 = 1602
    speed = response[0] + (response[1] << 8)

    # If speed is higher than 1032, return 1023 - speed, otherwise just return speed
    if speed > 1023:
      return 1023 - speed
    return speed

  def get_temperature(self, servo_id, model_name):
    """ Reads the servo's temperature value from its registers. """
    # Register Address and Length variables
    register_present_temperature = self.dynotools.getRegisterAddressByModel(model_name, "present_temperature")
    register_present_temperature_length = self.dynotools.getAddressSizeByModel(model_name, "present_temperature")

    # Read using present position
    raw_response = self.read(servo_id, register_present_temperature, register_present_temperature_length)
    response = raw_response[0]

    # Get the index 0, being the temperature in celsius
    temperature = response[0]
    return temperature

  def get_voltage(self, servo_id):
    """ Reads the servo's voltage. """
    response = self.read(servo_id, DXL_PRESENT_VOLTAGE, 1)
    if response:
      self.exception_on_error(response[4], servo_id, 'fetching supplied voltage')
    return response[5] / 10.0

  def get_current(self, servo_id, model_number, model_name):
    """ Reads the servo's current consumption (if supported by model) """
    # Make sure model is supported
    if not model_number in DXL_MODEL_TO_PARAMS:
      raise UnsupportedFeatureError(model_number, DXL_CURRENT_L)

    # Case of regular present current
    if DXL_CURRENT_L in DXL_MODEL_TO_PARAMS[model_number]['features']:
      # Register Address and Length variables
      register_present_current = self.dynotools.getRegisterAddressByModel(model_name, "present_current")
      register_present_current_length = self.dynotools.getAddressSizeByModel(model_name, "present_current")

      # Read using present position
      raw_response = self.read(servo_id, register_present_current, register_present_current_length)
      response = raw_response[0]
      
      # TODO: Either implement or remove error handling
      # if response:
        # self.exception_on_error(response[4], servo_id, 'fetching sensed current')

      # Caculate by bit shift left first index by eight, then add result to index 0
      # Example with [66, 6, 0, 0]: 66 << 8 = 1536; 1536 + 66 = 1602
      current = response[0] + (response[1] << 8)
      return 0.0045 * (current - 2048)

    # Case of sensed current
    if DXL_SENSED_CURRENT_L in DXL_MODEL_TO_PARAMS[model_number]['features']:
      # Register Address and Length variables
      register_present_current = self.dynotools.getRegisterAddressByModel(model_name, "sensed_current")
      register_present_current_length = self.dynotools.getAddressSizeByModel(model_name, "sensed_current")

      # Read using present position
      raw_response = self.read(servo_id, register_sensed_current, register_sensed_current_length)
      response = raw_response[0]

      # TODO: Either implement or remove error handling
      # if response:
        # self.exception_on_error(response[4], servo_id, 'fetching sensed current')
     
      # Caculate by bit shift left first index by eight, then add result to index 0
      # Example with [66, 6, 0, 0]: 66 << 8 = 1536; 1536 + 66 = 1602      
      current = response[0] + (response[1] << 8)
      return 0.01 * (current - 512)

    # No known way of supporting current
    else:
      raise UnsupportedFeatureError(model_number, DXL_CURRENT_L)

  def get_moving(self, servo_id, model_name):
    """ Reads the servo's temperature value from its registers. """
    # Register Address and Length variables
    register_moving = self.dynotools.getRegisterAddressByModel(model_name, "moving")
    register_moving_length = self.dynotools.getAddressSizeByModel(model_name, "moving")

    # Read using present position
    raw_response = self.read(servo_id, register_moving, register_moving_length)
    response = raw_response[0]

    # Get the index 0, being the temperature in celsius
    moving = response[0]
    return moving

  def get_feedback(self, servo_id, motor_info):
    """
    Returns the id, goal, position, error, speed, load, voltage, temperature
    and moving values from the specified servo.
    """
    # Get Model Name and Model Number (e.g., MX_64_T_2 - 311)
    model_number = motor_info[str(servo_id)]['model_number']
    model_name = self.dynotools.getModelNameByModelNumber(model_number)

    # Get feedback information
    goal = self.get_goal(servo_id, model_name)
    position = self.get_position(servo_id, model_name)
    error = position - goal
    speed = self.get_speed(servo_id, model_name)
    temperature = self.get_temperature(servo_id, model_name)
    moving = self.get_moving(servo_id, model_name)

    # Return above in a container form
    return { 'timestamp': 0,
             'id': servo_id,
             'goal': goal,
             'position': position,
             'error': error,
             'speed': speed,
             'load': 0,
             'voltage': 0,
             'temperature': temperature,
             'moving': bool(moving) }

    """
    # read in 17 consecutive bytes starting with low value for goal position
    response = self.read(servo_id, DXL_GOAL_POSITION_L, 17)

    # if response:
        # self.exception_on_error(response[4], servo_id, 'fetching full servo status')
    if len(response) == 31:
        # extract data values from the raw data
        goal = response[5] + (response[6] << 8)
        position = response[11] + (response[12] << 8)
        error = position - goal
        speed = response[13] + ( response[14] << 8)
        if speed > 1023: speed = 1023 - speed
        load_raw = response[15] + (response[16] << 8)
        load_direction = 1 if self.test_bit(load_raw, 10) else 0
        load = (load_raw & int('1111111111', 2)) / 1024.0
        if load_direction == 1: load = -load
        voltage = response[17] / 10.0
        temperature = response[18]
        moving = response[21]
        timestamp = response[-1]

        # return the data in a dictionary
        return { 'timestamp': timestamp,
                  'id': servo_id,
                  'goal': goal,
                  'position': position,
                  'error': error,
                  'speed': speed,
                  'load': load,
                  'voltage': voltage,
                  'temperature': temperature,
                  'moving': bool(moving) }
    """

  # TODO: look into if we need this function and below classes for error handling,
  #       and if so, how to implement them properly without serial calls.
  def exception_on_error(self, error_code, servo_id, command_failed):
        global exception
        exception = None
        ex_message = '[servo #%d on %s@%sbps]: %s failed' % (servo_id, self.port, self.baudrate, command_failed)

        if not isinstance(error_code, int):
            msg = 'Communcation Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, 0)
            return
        if not error_code & DXL_OVERHEATING_ERROR == 0:
            msg = 'Overheating Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & DXL_OVERLOAD_ERROR == 0:
            msg = 'Overload Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & DXL_INPUT_VOLTAGE_ERROR == 0:
            msg = 'Input Voltage Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_ANGLE_LIMIT_ERROR == 0:
            msg = 'Angle Limit Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_RANGE_ERROR == 0:
            msg = 'Range Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_CHECKSUM_ERROR == 0:
            msg = 'Checksum Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_INSTRUCTION_ERROR == 0:
            msg = 'Instruction Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)


class FatalErrorCodeError(Exception):
  def __init__(self, message, ec_const):
      Exception.__init__(self)
      self.message = message
      self.error_code = ec_const
  def __str__(self):
      return self.message


class UnsupportedFeatureError(Exception):
    def __init__(self, model_id, feature_id):
        Exception.__init__(self)
        if model_id in DXL_MODEL_TO_PARAMS:
            model = DXL_MODEL_TO_PARAMS[model_id]['name']
        else:
            model = 'Unknown'
        self.message = "Feature %d not supported by model %d (%s)" %(feature_id, model_id, model)
    def __str__(self):
        return self.message