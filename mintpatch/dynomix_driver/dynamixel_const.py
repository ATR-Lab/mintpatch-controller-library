"""
Dynamixel Constants
"""
# Control Table Constants
DXL_MODEL_NUMBER_L = 0
DXL_MODEL_NUMBER_H = 1
DXL_VERSION = 2
DXL_ID = 3
DXL_BAUD_RATE = 4
DXL_RETURN_DELAY_TIME = 5
DXL_CW_ANGLE_LIMIT_L = 6
DXL_CW_ANGLE_LIMIT_H = 7
DXL_CCW_ANGLE_LIMIT_L = 8
DXL_CCW_ANGLE_LIMIT_H = 9
DXL_DRIVE_MODE = 10
DXL_LIMIT_TEMPERATURE = 11
DXL_DOWN_LIMIT_VOLTAGE = 12
DXL_UP_LIMIT_VOLTAGE = 13
DXL_MAX_TORQUE_L = 14
DXL_MAX_TORQUE_H = 15
DXL_RETURN_LEVEL = 16
DXL_ALARM_LED = 17
DXL_ALARM_SHUTDOWN = 18
DXL_OPERATING_MODE = 19
DXL_DOWN_CALIBRATION_L = 20
DXL_DOWN_CALIBRATION_H = 21
DXL_UP_CALIBRATION_L = 22
DXL_UP_CALIBRATION_H = 23
DXL_TORQUE_ENABLE = 24
DXL_LED = 25
DXL_CW_COMPLIANCE_MARGIN = 26
DXL_CCW_COMPLIANCE_MARGIN = 27
DXL_CW_COMPLIANCE_SLOPE = 28
DXL_CCW_COMPLIANCE_SLOPE = 29
DXL_D_GAIN = 26
DXL_I_GAIN = 27
DXL_P_GAIN = 28
DXL_GOAL_POSITION_L = 30
DXL_GOAL_POSITION_H = 31
DXL_GOAL_SPEED_L = 32
DXL_GOAL_SPEED_H = 33
DXL_TORQUE_LIMIT_L = 34
DXL_TORQUE_LIMIT_H = 35
DXL_PRESENT_POSITION_L = 36
DXL_PRESENT_POSITION_H = 37
DXL_PRESENT_SPEED_L = 38
DXL_PRESENT_SPEED_H = 39
DXL_PRESENT_LOAD_L = 40
DXL_PRESENT_LOAD_H = 41
DXL_PRESENT_VOLTAGE = 42
DXL_PRESENT_TEMPERATURE = 43
DXL_REGISTERED_INSTRUCTION = 44
DXL_PAUSE_TIME = 45
DXL_MOVING = 46
DXL_LOCK = 47
DXL_PUNCH_L = 48
DXL_PUNCH_H = 49
DXL_SENSED_CURRENT_L = 56   # For EX-106
DXL_SENSED_CURRENT_H = 57
DXL_CURRENT_L = 68    # For MX-64 and up; different unit than EX-106
DXL_CURRENT_H = 69
DXL_TORQUE_CONTROL_MODE = 70
DXL_GOAL_TORQUE_L = 71
DXL_GOAL_TORQUE_H = 72
DXL_GOAL_ACCELERATION = 73

# Status Return Levels
DXL_RETURN_NONE = 0
DXL_RETURN_READ = 1
DXL_RETURN_ALL = 2

# Instruction Set
DXL_PING = 1
DXL_READ_DATA = 2
DXL_WRITE_DATA = 3
DXL_REG_WRITE = 4
DXL_ACTION = 5
DXL_RESET = 6
DXL_SYNC_WRITE = 131

# Broadcast Constant
DXL_BROADCAST = 254

# Error Codes
DXL_INSTRUCTION_ERROR = 64
DXL_OVERLOAD_ERROR = 32
DXL_CHECKSUM_ERROR = 16
DXL_RANGE_ERROR = 8
DXL_OVERHEATING_ERROR = 4
DXL_ANGLE_LIMIT_ERROR = 2
DXL_INPUT_VOLTAGE_ERROR = 1
DXL_NO_ERROR = 0

# Static parameters
DXL_MIN_COMPLIANCE_MARGIN = 0
DXL_MAX_COMPLIANCE_MARGIN = 255

DXL_MIN_COMPLIANCE_SLOPE = 1
DXL_MAX_COMPLIANCE_SLOPE = 254

# These are guesses as Dynamixel documentation doesn't have any info about it
DXL_MIN_PUNCH = 0
DXL_MAX_PUNCH = 255

DXL_MAX_SPEED_TICK = 1023                   # maximum speed in encoder units
DXL_MAX_TORQUE_TICK = 1023                  # maximum torque in encoder units

KGCM_TO_NM = 0.0980665                      # 1 kg-cm is that many N-m
RPM_TO_RADSEC = 0.104719755                 # 1 RPM is that many rad/sec


###########################################
# H54-200-S500-R 
###########################################
H54_200_S500_R_MODEL_NUMBER         = 54024
# Control Table Constants
H54_200_S500_R_MODEL                = 0
H54_200_S500_R_ID                   = 7
H54_200_S500_R_DRIVE_MODE           = 11          # Operating Mode
H54_200_S500_R_LIMIT_TEMPERATURE    = 21
H54_200_S500_R_LIMIT_MAX_VOLTAGE    = 22
H54_200_S500_R_LIMIT_MIN_VOLTAGE    = 24
H54_200_S500_R_LIMIT_ACCELERATION   = 26
H54_200_S500_R_LIMIT_TORQUE         = 30
H54_200_S500_R_LIMIT_VELOCITY       = 32

H54_200_S500_R_ANGLE_LIMIT_L        = 40
H54_200_S500_R_ANGLE_LIMIT_H        = 36

H54_200_S500_R_GOAL_POSITION        = 596
H54_200_S500_R_GOAL_VELOCITY        = 600
H54_200_S500_R_PRESENT_POSITION     = 611
H54_200_S500_R_PRESENT_VELOCITY     = 615
H54_200_S500_R_PRESENT_CURRENT      = 621
H54_200_S500_R_PRESENT_VOLTAGE      = 623
H54_200_S500_R_PRESENT_VOLTAGE_L    = 24
H54_200_S500_R_PRESENT_VOLTAGE_H    = 22
H54_200_S500_R_PRESENT_TEMPERATURE  = 625

H54_200_S500_R_MODEL_LENGTH                 = 2
H54_200_S500_R_ID_LENGTH                    = 1
H54_200_S500_R_DRIVE_MODE_LENGTH            = 1          # Operating Mode
H54_200_S500_R_LIMIT_TEMPERATURE_LENGTH     = 1
H54_200_S500_R_LIMIT_MAX_VOLTAGE_LENGTH     = 2
H54_200_S500_R_LIMIT_MIN_VOLTAGE_LENGTH     = 2
H54_200_S500_R_LIMIT_ACCELERATION_LENGTH    = 4
H54_200_S500_R_LIMIT_TORQUE_LENGTH          = 2
H54_200_S500_R_LIMIT_VELOCITY_LENGTH        = 4
H54_200_S500_R_ANGLE_LIMIT_L_LENGTH         = 4
H54_200_S500_R_ANGLE_LIMIT_H_LENGTH         = 4
H54_200_S500_R_GOAL_POSITION_LENGTH         = 4
H54_200_S500_R_GOAL_VELOCITY_LENGTH         = 4
H54_200_S500_R_PRESENT_POSITION_LENGTH      = 4
H54_200_S500_R_PRESENT_VELOCITY_LENGTH      = 4
H54_200_S500_R_PRESENT_CURRENT_LENGTH       = 2
H54_200_S500_R_PRESENT_VOLTAGE_LENGTH       = 2
H54_200_S500_R_PRESENT_VOLTAGE_L_LENGTH     = 2
H54_200_S500_R_PRESENT_VOLTAGE_H_LENGTH     = 2
H54_200_S500_R_PRESENT_TEMPERATURE_LENGTH   = 2

###########################################
# XM540-W270-T 
###########################################
XM540_W270_T_NUMBER               = 1120
# Control Table Constants
XM540_W270_T_MODEL                = 0
XM540_W270_T_ID                   = 7
XM540_W270_T_DRIVE_MODE           = 10      # Operating Mode
XM540_W270_T_LIMIT_TEMPERATURE    = 31
XM540_W270_T_LIMIT_MAX_VOLTAGE    = 32
XM540_W270_T_LIMIT_MIN_VOLTAGE    = 34
XM540_W270_T_LIMIT_ACCELERATION   = 108     # NOTE: NO LIMIT ACCELERATION http://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#profile-acceleration
XM540_W270_T_LIMIT_TORQUE         = None    # TODO: Search if this is true  
XM540_W270_T_LIMIT_VELOCITY       = 44
XM540_W270_T_GOAL_POSITION        = 116
XM540_W270_T_GOAL_VELOCITY        = 104
XM540_W270_T_PRESENT_POSITION     = 132
XM540_W270_T_PRESENT_VELOCITY     = 128
XM540_W270_T_PRESENT_CURRENT      = 126
XM540_W270_T_PRESENT_TEMPERATURE  = 146

XM540_W270_T_MODEL_LENGTH                = 2
XM540_W270_T_DRIVE_MODE_LENGTH           = 1    # Operating Mode
XM540_W270_T_LIMIT_TEMPERATURE_LENGTH    = 1
XM540_W270_T_LIMIT_MAX_VOLTAGE_LENGTH    = 2
XM540_W270_T_LIMIT_MIN_VOLTAGE_LENGTH    = 2
XM540_W270_T_LIMIT_ACCELERATION_LENGTH   = 4    # NOTE: http://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#profile-acceleration
XM540_W270_T_LIMIT_TORQUE_LENGTH         = None
XM540_W270_T_LIMIT_VELOCITY_LENGTH       = 4
XM540_W270_T_GOAL_POSITION_LENGTH        = 4
XM540_W270_T_GOAL_VELOCITY_LENGTH        = 4
XM540_W270_T_PRESENT_POSITION_LENGTH     = 4
XM540_W270_T_PRESENT_VELOCITY_LENGTH     = 4
XM540_W270_T_PRESENT_CURRENT_LENGTH      = 2
XM540_W270_T_PRESENT_TEMPERATURE_LENGTH  = 1


###########################################
# MX-64-T-2
###########################################  MX_64_T_2
MX_64_T_2_NUMBER               = 311
# Control Table Constants
MX_64_T_2_MODEL                = 0
MX_64_T_2_ID                   = 7
MX_64_T_2_DRIVE_MODE           = 10      # Operating Mode
MX_64_T_2_LIMIT_TEMPERATURE    = 31
MX_64_T_2_LIMIT_VOLTAGE_H      = 32
MX_64_T_2_LIMIT_VOLTAGE_L      = 34
MX_64_T_2_LIMIT_ACCELERATION   = 40    
MX_64_T_2_LIMIT_TORQUE         = None     
MX_64_T_2_LIMIT_VELOCITY       = 44     
MX_64_T_2_ANGLE_LIMIT_L        = 52
MX_64_T_2_ANGLE_LIMIT_H        = 48
MX_64_T_2_GOAL_POSITION        = 116
MX_64_T_2_GOAL_VELOCITY        = 104
MX_64_T_2_PRESENT_POSITION     = 132
MX_64_T_2_PRESENT_VELOCITY     = 128
MX_64_T_2_PRESENT_CURRENT      = 126
MX_64_T_2_PRESENT_VOLTAGE      = 144
MX_64_T_2_PRESENT_TEMPERATURE  = 146

MX_64_T_2_MODEL_LENGTH                = 2
MX_64_T_2_DRIVE_MODE_LENGTH           = 1    # Operating Mode
MX_64_T_2_LIMIT_TEMPERATURE_LENGTH    = 1
MX_64_T_2_LIMIT_VOLTAGE_H_LENGTH    = 2
MX_64_T_2_LIMIT_VOLTAGE_L_LENGTH    = 2
MX_64_T_2_LIMIT_ACCELERATION_LENGTH   = 4    # NOTE: http://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#profile-acceleration
MX_64_T_2_LIMIT_TORQUE_LENGTH         = None
MX_64_T_2_LIMIT_VELOCITY_LENGTH       = 4
MX_64_T_2_ANGLE_LIMIT_L_LENGTH        = 4
MX_64_T_2_ANGLE_LIMIT_H_LENGTH        = 4
MX_64_T_2_GOAL_POSITION_LENGTH        = 4
MX_64_T_2_GOAL_VELOCITY_LENGTH        = 4
MX_64_T_2_PRESENT_POSITION_LENGTH     = 4
MX_64_T_2_PRESENT_VELOCITY_LENGTH     = 4
MX_64_T_2_PRESENT_CURRENT_LENGTH      = 2
MX_64_T_2_PRESENT_VOLTAGE_LENGTH      = 2
MX_64_T_2_PRESENT_TEMPERATURE_LENGTH  = 1


###########################################
# MX-106-T-2
###########################################
MX_106_T_2_NUMBER               = 321
# Control Table Constants
MX_106_T_2_MODEL                = 0
MX_106_T_2_ID                   = 7
MX_106_T_2_DRIVE_MODE           = 0      # Operating Mode
MX_106_T_2_LIMIT_TEMPERATURE    = 72
MX_106_T_2_LIMIT_VOLTAGE_H    = 140
MX_106_T_2_LIMIT_VOLTAGE_L    = 60
MX_106_T_2_LIMIT_ACCELERATION   = 40    
MX_106_T_2_LIMIT_TORQUE         = None     
MX_106_T_2_LIMIT_VELOCITY       = 44   
MX_106_T_2_ANGLE_LIMIT_L        = 52
MX_106_T_2_ANGLE_LIMIT_H        = 48
MX_106_T_2_GOAL_POSITION        = 116
MX_106_T_2_GOAL_VELOCITY        = 104
MX_106_T_2_PRESENT_POSITION     = 132
MX_106_T_2_PRESENT_VELOCITY     = 128
MX_106_T_2_PRESENT_CURRENT      = 126
MX_106_T_2_PRESENT_VOLTAGE      = 144
MX_106_T_2_PRESENT_TEMPERATURE  = 146

MX_106_T_2_MODEL_LENGTH                = 2
MX_106_T_2_DRIVE_MODE_LENGTH           = 1    # Operating Mode
MX_106_T_2_LIMIT_TEMPERATURE_LENGTH    = 1
MX_106_T_2_LIMIT_VOLTAGE_H_LENGTH    = 2
MX_106_T_2_LIMIT_VOLTAGE_L_LENGTH    = 2
MX_106_T_2_LIMIT_ACCELERATION_LENGTH   = 4    # NOTE: http://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#profile-acceleration
MX_106_T_2_LIMIT_TORQUE_LENGTH         = None
MX_106_T_2_LIMIT_VELOCITY_LENGTH       = 4
MX_106_T_2_ANGLE_LIMIT_L_LENGTH        = 4
MX_106_T_2_ANGLE_LIMIT_H_LENGTH        = 4
MX_106_T_2_GOAL_POSITION_LENGTH        = 4
MX_106_T_2_GOAL_VELOCITY_LENGTH        = 4
MX_106_T_2_PRESENT_POSITION_LENGTH     = 4
MX_106_T_2_PRESENT_VELOCITY_LENGTH     = 4
MX_106_T_2_PRESENT_CURRENT_LENGTH      = 2
MX_106_T_2_PRESENT_TEMPERATURE_LENGTH  = 1
MX_106_T_2_PRESENT_VOLTAGE_LENGTH      = 2


###########################################
# XL430_w250_T_2
###########################################
XL430_w250_T_2_NUMBER               = 1060
# Control Table Constants
XL430_w250_T_2_MODEL                = 0
XL430_w250_T_2_ID                   = 7
XL430_w250_T_2_DRIVE_MODE           = 10      # Operating Mode
XL430_w250_T_2_LIMIT_TEMPERATURE    = 31
XL430_w250_T_2_LIMIT_VOLTAGE_H    = 32
XL430_w250_T_2_LIMIT_VOLTAGE_L    = 34
XL430_w250_T_2_LIMIT_ACCELERATION   = 40    
XL430_w250_T_2_LIMIT_TORQUE         = None     
XL430_w250_T_2_LIMIT_VELOCITY       = 44   
XL430_w250_T_2_ANGLE_LIMIT_L        = 52
XL430_w250_T_2_ANGLE_LIMIT_H        = 48
XL430_w250_T_2_GOAL_POSITION        = 116
XL430_w250_T_2_GOAL_VELOCITY        = 104
XL430_w250_T_2_PRESENT_POSITION     = 132
XL430_w250_T_2_PRESENT_VELOCITY     = 128
XL430_w250_T_2_PRESENT_CURRENT      = 126
XL430_w250_T_2_PRESENT_VOLTAGE      = 144
XL430_w250_T_2_PRESENT_TEMPERATURE  = 146

XL430_w250_T_2_MODEL_LENGTH                = 2
XL430_w250_T_2_DRIVE_MODE_LENGTH           = 1    # Operating Mode
XL430_w250_T_2_LIMIT_TEMPERATURE_LENGTH    = 1
XL430_w250_T_2_LIMIT_VOLTAGE_H_LENGTH    = 2
XL430_w250_T_2_LIMIT_VOLTAGE_L_LENGTH    = 2
XL430_w250_T_2_LIMIT_ACCELERATION_LENGTH   = 4    # NOTE: http://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#profile-acceleration
XL430_w250_T_2_LIMIT_TORQUE_LENGTH         = None
XL430_w250_T_2_LIMIT_VELOCITY_LENGTH       = 4
XL430_w250_T_2ANGLE_LIMIT_L_LENGTH        = 4
XL430_w250_T_2_ANGLE_LIMIT_H_LENGTH        = 4
XL430_w250_T_2_GOAL_POSITION_LENGTH        = 4
XL430_w250_T_2_GOAL_VELOCITY_LENGTH        = 4
XL430_w250_T_2_PRESENT_POSITION_LENGTH     = 4
XL430_w250_T_2_PRESENT_VELOCITY_LENGTH     = 4
XL430_w250_T_2_PRESENT_CURRENT_LENGTH      = 2
XL430_w250_T_2_PRESENT_TEMPERATURE_LENGTH  = 1
XL430_w250_T_2_PRESENT_VOLTAGE_LENGTH      = 2

# maximum holding torque is in N-m per volt
# maximum velocity is in rad/sec per volt
DXL_MODEL_TO_PARAMS = \
{

  321: {
    'name':               'MX_106_T_2',
    'encoder_resolution': 4096,
    'range_degrees':      360.0,
    'torque_per_volt':    8.4 / 12.0,                       #  8.4 NM @ 12V
    'velocity_per_volt':  (45 * RPM_TO_RADSEC) / 12.0,      #  45 RPM @ 12.0V
    'rpm_per_tick':       0.114,
    'features':           [DXL_CURRENT_L, DXL_TORQUE_CONTROL_MODE, DXL_GOAL_ACCELERATION]
  },
  54024: {
    "name": "H54_200_S500_R_2",
    'encoder_resolution': 501923,
    'range_degrees': 360,        # In case of the Position Control Mode(Joint Mode) that rotates less than 360 degrees, any invalid Homing Offset(20) values will be ignored(valid range : -1,024 ~ 1,024).
    'torque_per_volt': 44.7 / 12.0,  # TODO: Needs to be completed, values not found in specs sheets (voltages might be different)
    'velocity_per_volt': (29.0 * RPM_TO_RADSEC) / 12.0, # TODO: Needs to be completed, values not found in specs sheets (voltages might be different)
    'rpm_per_tick': 0.00199234,
    'features': [DXL_CURRENT_L]
  },
  311: {
    "name": "MX_64_T_2",
    'encoder_resolution': 4096,
    'range_degrees':      360.0,
    'torque_per_volt':    6.0 / 12.0,                       #  6 NM @ 12V
    'velocity_per_volt':  (63 * RPM_TO_RADSEC) / 12.0,      #  63 RPM @ 12.0V
    'rpm_per_tick':       0.114,
    'features':           [DXL_CURRENT_L, DXL_TORQUE_CONTROL_MODE, DXL_GOAL_ACCELERATION]
  },
  1120: {
    'name': 'XM540_w270_R_2',
    'encoder_resolution': 4096,
    'range_degree': 360,
    'torque_per_volt': 10.6 / 12.0,
    'velocity_per_volt': (30 * RPM_TO_RADSEC) / 12.0,
    'features': [DXL_CURRENT_L]
  },


# TODO: Implement rest of motor containers
  1060: { 
    'name': 'XL430_w250_T_2',
    'encoder_resolution': 4096,
    'range_degree': 360,
    'torque_per_volt': 1.5 / 11.0,        # 1.4 [Nm] (at 11.1 [V], 1.3 [A])::::: 1.5 [Nm] (at 12.0 [V], 1.4 [A])
    'velocity_per_volt': (57 * RPM_TO_RADSEC) / 12.0,  # 57 [rev/min] (at 11.1 [V])
    'features': []
  },
  113: { 
    'name':               'DX-113',
    'encoder_resolution': 1024,
    'range_degrees':      300.0,
    'torque_per_volt':    1.0 / 12.0,                       #  1.0 NM @ 12V
    'velocity_per_volt':  (54 * RPM_TO_RADSEC) / 12.0,      #  54 RPM @ 12V
    'rpm_per_tick':       0.111,
    'features':           []
  },
  116: { 
    'name':               'DX-116',
    'encoder_resolution': 1024,
    'range_degrees':      300.0,
    'torque_per_volt':    2.1 / 12.0,                       #  2.1 NM @ 12V
    'velocity_per_volt':  (78 * RPM_TO_RADSEC) / 12.0,      #  78 RPM @ 12V
    'rpm_per_tick':       0.111,
    'features':           []
  },
  117: { 'name':               'DX-117',
    'encoder_resolution': 1024,
    'range_degrees':      300.0,
    'torque_per_volt':    3.7 / 18.5,                       #  3.7 NM @ 18.5V
    'velocity_per_volt':  (85 * RPM_TO_RADSEC) / 18.5,      #  85 RPM @ 18.5V
    'rpm_per_tick':       0.111,
    'features':           []
  },
  300: { 
    'name':               'AX-12W',
    'encoder_resolution': 1024,
    'range_degrees':      300.0,
    'torque_per_volt':    0.2 / 12.0,                       # 0.2 NM @ 12V
    'velocity_per_volt':  (470 * RPM_TO_RADSEC) / 12.0,     # 470 RPM @ 12V
    'rpm_per_tick':       0.111,
    'features':           []
  },
  18: { 
    'name':               'AX-18',
    'encoder_resolution': 1024,
    'range_degrees':      300.0,
    'torque_per_volt':    1.8 / 12.0,                       #  1.8 NM @ 12V
    'velocity_per_volt':  (97 * RPM_TO_RADSEC) / 12.0,      #  97 RPM @ 12V
    'rpm_per_tick':       0.111,
    'features':           []
  },
  10: { 
    'name':               'RX-10',
    'encoder_resolution': 1024,
    'range_degrees':      300.0,
    'torque_per_volt':    1.3 / 12.0,                       #  1.3 NM @ 12V
    'velocity_per_volt':  (54 * RPM_TO_RADSEC) / 12.0,      #  54 RPM @ 12V
    'rpm_per_tick':       0.111,
    'features':           []
  },
  24: { 
    'name':               'RX-24',
    'encoder_resolution': 1024,
    'range_degrees':      300.0,
    'torque_per_volt':    2.6 / 12.0,                       #  2.6 NM @ 12V
    'velocity_per_volt':  (126 * RPM_TO_RADSEC) / 12.0,     # 126 RPM @ 12V
    'rpm_per_tick':       0.111,
    'features':           []
  },
  28: { 'name':               'RX-28',
    'encoder_resolution': 1024,
    'range_degrees':      300.0,
    'torque_per_volt':    3.7 / 18.5,                       #  3.7 NM @ 18.5V
    'velocity_per_volt':  (85 * RPM_TO_RADSEC) / 18.5,      #  85 RPM @ 18.5V
    'rpm_per_tick':       0.111,
    'features':           []
  },
  64: { 
    'name':               'RX-64',
    'encoder_resolution': 1024,
    'range_degrees':      300.0,
    'torque_per_volt':    5.3 / 18.5,                       #  5.3 NM @ 18.5V
    'velocity_per_volt':  (64 * RPM_TO_RADSEC) / 18.5,      #  64 RPM @ 18.5V
    'rpm_per_tick':       0.111,
    'features':           []
  },
  106: { 
    'name':               'EX-106',
    'encoder_resolution': 4096,
    'range_degrees':      250.92,
    'torque_per_volt':    10.9 / 18.5,                      # 10.9 NM @ 18.5V
    'velocity_per_volt':  (91 * RPM_TO_RADSEC) / 18.5,      #  91 RPM @ 18.5V
    'rpm_per_tick':       0.111,
    'features':           [DXL_SENSED_CURRENT_L]
  },     
  107: { 
    'name':               'EX-106+',
    'encoder_resolution': 4096,
    'range_degrees':      250.92,
    'torque_per_volt':    10.9 / 18.5,                      # 10.9 NM @ 18.5V
    'velocity_per_volt':  (91 * RPM_TO_RADSEC) / 18.5,      #  91 RPM @ 18.5V
    'rpm_per_tick':       0.111,
    'features':           [DXL_SENSED_CURRENT_L]
  },
  360: { 
    'name':               'MX-12W',
    'encoder_resolution': 4096,
    'range_degrees':      360.0,
    'torque_per_volt':    0.2 / 12.0,                        #  torque not specified!
    'velocity_per_volt':  (470 * RPM_TO_RADSEC) / 12.0,      #  470 RPM @ 12.0V
    'rpm_per_tick':       0.114,
    'features':           [DXL_GOAL_ACCELERATION]
  },
  29: { 
    'name':               'MX-28',
    'encoder_resolution': 4096,
    'range_degrees':      360.0,
    'torque_per_volt':    2.5 / 12.0,                       #  2.5 NM @ 12V
    'velocity_per_volt':  (55 * RPM_TO_RADSEC) / 12.0,      #  54 RPM @ 12.0V
    'rpm_per_tick':       0.114,
    'features':           [DXL_GOAL_ACCELERATION]
  },
  310: { 
    'name':               'MX-64',
    'encoder_resolution': 4096,
    'range_degrees':      360.0,
    'torque_per_volt':    6.0 / 12.0,                       #  6 NM @ 12V
    'velocity_per_volt':  (63 * RPM_TO_RADSEC) / 12.0,      #  63 RPM @ 12.0V
    'rpm_per_tick':       0.114,
    'features':           [DXL_CURRENT_L, DXL_TORQUE_CONTROL_MODE, DXL_GOAL_ACCELERATION]
  },
  320: { 
    'name':               'MX-106',
    'encoder_resolution': 4096,
    'range_degrees':      360.0,
    'torque_per_volt':    8.4 / 12.0,                       #  8.4 NM @ 12V
    'velocity_per_volt':  (45 * RPM_TO_RADSEC) / 12.0,      #  45 RPM @ 12.0V
    'rpm_per_tick':       0.114,
    'features':           [DXL_CURRENT_L, DXL_TORQUE_CONTROL_MODE, DXL_GOAL_ACCELERATION]
  },
}
