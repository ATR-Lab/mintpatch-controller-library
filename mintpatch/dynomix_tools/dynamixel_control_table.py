

# Assumes it's 2 bytes and address = 0
MODEL_NUMBER_2_MOTOR_NAME = {
  321: {
    "name": "MX_106_2"
  },
  54024: {
    "name": "H54_200_S500_R_2"
  },
  311: {
    "name": "MX_64_T_2",
  },
  1120: {
    "name": "XM540_w270_R_2"
  },
  1060: {
    "name": "XL430_w250_T_2"
  }
}

MOTOR_CONTROL_TABLE = {
  "MX_106_2": {
    "goal_position": {
      "address": 116,
      "size": 4
    },
    "torque_enable": {
      "address": 64,
      "size": 1
    },
    "torque_limit": {
      "address": 34,
      "size": 1
    },
    "profile_velocity": {
      "address":112,
      "size": 4
    },
    "summary_size": 31 
  },
  "H54_200_S500_R_2": {
    "goal_position": {
      "address": 596,
      "size": 4
    },
    "torque_enable": {
      "address": 562,
      "size": 1
    },
    "torque_limit": {
      "address": 30,
      "size": 2
    },
    "profile_velocity": {
      "address":600,
      "size": 4
    },
    "summary_size": 31 
  },
  "MX_64_T_2": {
    "goal_position": {
      "address": 116,
      "size": 4
    },
    "torque_enable": {
      "address": 64,
      "size": 1
    },
    "torque_Limit": {
      "address": None,
      "size": None
    },
    "profile_velocity": {
      "address":112,
      "size": 4
    },
    "summary_size": 31 
  },
  "XM540_w270_R_2": {
    "goal_position": {
      "address": 116,
      "size": 4
    },
    "torque_enable": {
      "address": 64,
      "size": 1
    },
    "torque_limit": {
      "address": None,
      "size": None
    },
    "profile_velocity": {
      "address":112,
      "size": 4
    },
    "summary_size": 31 
  },
  "XL430_w250_T_2": {
    "goal_position": {
      "address": 116,
      "size": 4
    },
    "torque_enable": {
      "address": 64,
      "size": 1
    },
    "torque_limit": {
      "address": 30,
      "size": 2
    },
    "profile_velocity": {
      "address":112,
      "size": 4
    },
    "summary_size": 31 
  }
}

# DynamixelInfo:
s_Acceleration_Limit = "Acceleration_Limit"
s_Alarm_LED = "Alarm_LED"
s_Baud_Rate = "Baud_Rate"
s_Bus_Watchdog = "Bus_Watchdog"
s_CCW_Angle_Limit = "CCW_Angle_Limit"
s_CCW_Compliance_Margin = "CCW_Compliance_Margin"
s_CCW_Compliance_Slope = "CCW_Compliance_Slope"
s_Control_Mode = "Control_Mode"
s_Current = "Current"
s_Current_Limit = "Current_Limit"
s_CW_Angle_Limit = "CW_Angle_Limit"
s_CW_Compliance_Margin = "CW_Compliance_Margin"
s_CW_Compliance_Slope = "CW_Compliance_Slope"
s_D_gain = "D_gain"
s_Drive_Mode = "Drive_Mode"
s_External_Port_Mode_1 = "External_Port_Mode_1"
s_External_Port_Mode_2 = "External_Port_Mode_2"
s_External_Port_Mode_3 = "External_Port_Mode_3"
s_External_Port_Mode_4 = "External_Port_Mode_4"
s_Feedforward_1st_Gain = "Feedforward_1st_Gain"
s_Feedforward_2nd_Gain = "Feedforward_2nd_Gain"
s_Firmware_Version = "Firmware_Version"
s_Goal_Acceleration = "Goal_Acceleration"
s_Goal_Current = "Goal_Current"
s_Goal_Position = "Goal_Position"
s_Goal_PWM = "Goal_PWM"
s_Goal_Torque = "Goal_Torque"
s_Goal_Velocity = "Goal_Velocity"
s_Hardware_Error_Status = "Hardware_Error_Status"
s_Homing_Offset = "Homing_Offset"
s_I_gain = "I_gain"
s_ID = "ID"
s_LED = "LED"
s_LED_BLUE = "LED_BLUE"
s_LED_GREEN = "LED_GREEN"
s_LED_RED = "LED_RED"
s_Lock = "Lock"
s_Max_Position_Limit = "Max_Position_Limit"
s_Max_Torque = "Max_Torque"
s_Max_Voltage_Limit = "Max_Voltage_Limit"
s_Min_Position_Limit = "Min_Position_Limit"
s_Min_Voltage_Limit = "Min_Voltage_Limit"
s_Model_Number = "Model_Number"
s_Moving = "Moving"
s_Moving_Speed = "Moving_Speed"
s_Moving_Status = "Moving_Status"
s_Moving_Threshold = "Moving_Threshold"
s_Multi_Turn_Offset = "Multi_Turn_Offset"
s_Operating_Mode = "Operating_Mode"
s_P_gain = "P_gain"
s_Position_D_Gain = "Position_D_Gain"
s_Position_I_Gain = "Position_I_Gain"
s_Position_P_Gain = "Position_P_Gain"
s_Position_Trajectory = "Position_Trajectory"
s_Present_Current = "Present_Current"
s_Present_Input = "Present_Input"
s_Present_Input_Voltage = "Present_Input_Voltage"
s_Present_Load = "Present_Load"
s_Present_Position = "Present_Position"
s_Present_PWM = "Present_PWM"
s_Present_Speed = "Present_Speed"
s_Present_Temperature = "Present_Temperature"
s_Present_Velocity = "Present_Velocity"
s_Present_Voltage = "Present_Voltage"
s_Profile_Acceleration = "Profile_Acceleration"
s_Profile_Velocity = "Profile_Velocity"
s_Protocol_Version = "Protocol_Version"
s_Punch = "Punch"
s_PWM_Limit = "PWM_Limit"
s_Realtime_Tick = "Realtime_Tick"
s_Registered = "Registered"
s_Registered_Instruction = "Registered_Instruction"
s_Resolution_Divider = "Resolution_Divider"
s_Return_Delay_Time = "Return_Delay_Time"
s_Secondary_ID = "Secondary_ID"
s_Sensored_Current = "Sensored_Current"
s_Shutdown = "Shutdown"
s_Status_Return_Level = "Status_Return_Level"
s_Temperature_Limit = "Temperature_Limit"
s_Torque_Control_Mode_Enable = "Torque_Control_Mode_Enable"
s_Torque_Enable = "Torque_Enable"
s_Torque_Limit = "Torque_Limit"
s_Velocity_I_Gain = "Velocity_I_Gain"
s_Velocity_Limit = "Velocity_Limit"
s_Velocity_P_Gain = "Velocity_P_Gain"
s_Velocity_Trajectory = "Velocity_Trajectory"

# "address", "size", default 
CONTROL_TABLE_EXTMX2 = {
    s_Model_Number: { "address": 0, "size": 2},
    s_Firmware_Version: { "address": 6, "size": 1},
    s_ID: {"address": 7, "size": 1},
    s_Baud_Rate: { "address": 8, "size": 1},
    s_Return_Delay_Time: { "address": 9, "size": 1},
    s_Drive_Mode: {"address": 10, "size": 1},
    s_Operating_Mode: {"address": 11, "size": 1},
    s_Secondary_ID: { "address": 12, "size": 1},
    s_Protocol_Version: { "address": 13, "size": 1},
    s_Homing_Offset: { "address": 20, "size": 4},
    s_Moving_Threshold: { "address": 24, "size": 4},
    s_Temperature_Limit: { "address": 31, "size": 1},
    s_Max_Voltage_Limit: { "address": 32, "size": 2},
    s_Min_Voltage_Limit: { "address": 34, "size": 2},
    s_PWM_Limit: { "address": 36, "size": 2},
    s_Current_Limit: { "address": 38, "size": 2},
    s_Acceleration_Limit: { "address": 40, "size": 4},
    s_Velocity_Limit: { "address": 44, "size": 4},
    s_Max_Position_Limit: { "address": 48, "size": 4},
    s_Min_Position_Limit: { "address": 52, "size": 4},
    s_Shutdown: { "address": 63, "size": 1},

    s_Torque_Enable: { "address": 64, "size": 1},
    s_LED: { "address": 65, "size": 1},
    s_Status_Return_Level: { "address": 68, "size": 1},
    s_Registered_Instruction: { "address": 69, "size": 1},
    s_Hardware_Error_Status: {"address": 70, "size": 1},
    s_Velocity_I_Gain: { "address": 76, "size": 2},
    s_Velocity_P_Gain: { "address": 78, "size": 2},
    s_Position_D_Gain: { "address": 80, "size": 2},
    s_Position_I_Gain: { "address": 82, "size": 2},
    s_Position_P_Gain: { "address": 84, "size": 2},
    s_Feedforward_2nd_Gain: { "address": 88, "size": 2},
    s_Feedforward_1st_Gain: { "address": 90, "size": 2},
    s_Bus_Watchdog: { "address": 98, "size": 1},
    s_Goal_PWM: { "address": 100, "size": 2},
    s_Goal_Current: { "address": 102, "size": 2},
    s_Goal_Velocity: { "address": 104, "size": 4},
    s_Profile_Acceleration: {"address": 108, "size": 4},
    s_Profile_Velocity: { "address": 112, "size": 4},
    s_Goal_Position: { "address": 116, "size": 4},
    s_Realtime_Tick: { "address": 120, "size": 2},
    s_Moving: {"address": 122, "size": 1},
    s_Moving_Status: { "address": 123, "size": 1},
    s_Present_PWM: { "address": 124, "size": 2},
    s_Present_Current: { "address": 126, "size": 2},
    s_Present_Velocity: { "address": 128, "size": 4},
    s_Present_Position: { "address": 132, "size": 4},
    s_Velocity_Trajectory: { "address": 136, "size": 4},
    s_Position_Trajectory: { "address": 140, "size": 4},
    s_Present_Input_Voltage: { "address": 144, "size": 2},
    s_Present_Temperature: { "address": 146, "size": 1}
}

# CONTROL_TABLE_EXTMX2 = {
#     s_Model_Number: {0, s_Model_Numb, 2},
#     s_Firmware_Version: {6, s_Firmware_Versi, 1},
#     s_ID: {7, s_, 1},
#     s_Baud_Rate: {8, s_Baud_Ra, 1},
#     s_Return_Delay_Time: {9, s_Return_Delay_Ti, 1},
#     s_Drive_Mode: { 10, s_Drive_Mo, 1},
#     s_Operating_Mode: { 11, s_Operating_Mo, 1},
#     s_Secondary_ID: { 12, s_Secondary_, 1},
#     s_Protocol_Version: { 13, s_Protocol_Versi, 1},
#     s_Homing_Offset: { 20, s_Homing_Offs, 4},
#     s_Moving_Threshold: { 24, s_Moving_Thresho, 4},
#     s_Temperature_Limit: { 31, s_Temperature_Lim, 1},
#     s_Max_Voltage_Limit: { 32, s_Max_Voltage_Lim, 2},
#     s_Min_Voltage_Limit: { 34, s_Min_Voltage_Lim, 2},
#     s_PWM_Limit: { 36, s_PWM_Lim, 2},
#     s_Current_Limit: { 38, s_Current_Lim, 2},
#     s_Acceleration_Limit: { 40, s_Acceleration_Lim, 4},
#     s_Velocity_Limit: { 44, s_Velocity_Lim, 4},
#     s_Max_Position_Limit: { 48, s_Max_Position_Lim, 4},
#     s_Min_Position_Limit: { 52, s_Min_Position_Lim, 4},
#     s_Shutdown: { 63, s_Shutdo, 1},

#     s_Torque_Enable: { 64, s_Torque_Enab, 1},
#     s_LED: { 65, s_L, 1},
#     s_Status_Return_Level: { 68, s_Status_Return_Lev, 1},
#     s_Registered_Instruction: { 69, s_Registered_Instructi, 1},
#     s_Hardware_Error_Status: {70, s_Hardware_Error_Stat, 1},
#     s_Velocity_I_Gain: { 76, s_Velocity_I_Ga, 2},
#     s_Velocity_P_Gain: { 78, s_Velocity_P_Ga, 2},
#     s_Position_D_Gain: { 80, s_Position_D_Ga, 2},
#     s_Position_I_Gain: { 82, s_Position_I_Ga, 2},
#     s_Position_P_Gain: { 84, s_Position_P_Ga, 2},
#     s_Feedforward_2nd_Gain: { 88, s_Feedforward_2nd_Ga, 2},
#     s_Feedforward_1st_Gain: { 90, s_Feedforward_1st_Ga, 2},
#     s_Bus_Watchdog: { 98, s_Bus_Watchd, 1},
#     s_Goal_PWM: { 100, s_Goal_P, 2},
#     s_Goal_Current: { 102, s_Goal_Curre, 2},
#     s_Goal_Velocity: { 104, s_Goal_Veloci, 4},
#     s_Profile_Acceleration: {108, s_Profile_Accelerati, 4},
#     s_Profile_Velocity: { 112, s_Profile_Veloci, 4},
#     s_Goal_Position: { 116, s_Goal_Positi, 4},
#     s_Realtime_Tick: { 120, s_Realtime_Ti, 2},
#     s_Moving: {122, s_Movi, 1},
#     s_Moving_Status: { 123, s_Moving_Stat, 1},
#     s_Present_PWM: { 124, s_Present_P, 2},
#     s_Present_Current: { 126, s_Present_Curre, 2},
#     s_Present_Velocity: { 128, s_Present_Veloci, 4},
#     s_Present_Position: { 132, s_Present_Positi, 4},
#     s_Velocity_Trajectory: { 136, s_Velocity_Trajecto, 4},
#     s_Position_Trajectory: { 140, s_Position_Trajecto, 4},
#     s_Present_Input_Voltage: { 144, s_Present_Input_Volta, 2},
#     s_Present_Temperature: { 146, s_Present_Temperatu, 1}
# }

info_EXTMX2 = {
  0.11,
  0,
  2048,
  4096,
  -3.14159265, 
  3.14159265
}