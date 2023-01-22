// This file is for Arduino and Python Interface be careful when adding stuff
// "// some_text" will be a python dictionary key until next "// other_text". 
// Do not put comments at the end of #define lines

// serial_settings
#define BAUD_RATE                           250000
#define SERIAL_BUFFER_LENGTH                256
#define STATUS_MESSAGE_LENGTH               54
#define SERIAL_TIMEOUT_MS                   100
#define STX                                 0x02
#define ETX                                 0x03
#define ACK                                 0x06
#define NAK                                 0x15

// status_message
#define MOTOR_STATUS_UPDATE_INTERVAL        250
#define MOTOR_PULSES_UPDATE_INTERVAL        500
#define LOAD_CELL_UPDATE_INTERVAL           250
#define STATUS_MINIMUM_LENGTH               2
#define BYTES_PER_LOADCELL                  4
#define BYTES_PER_MOTOR                     6

// status_message_bits
#define STATUS_DIRECTION_BIT                0
#define STATUS_FAULT_BIT                    1
#define STATUS_PAUSED_BIT                   2
#define STATUS_RAMPING_BIT                  3
#define STATUS_SPARE_BIT                    4
#define STATUS_ENABLED_BIT                  5
#define STATUS_RUNNING_BIT                  6
#define STATUS_SLEEP_BIT                    7

// message_types
#define MOTOR_STATUS_MESSAGE_ID             0xFF
#define MOTOR_PULSES_MESSAGE_ID             0xFE
#define LOAD_CELL_STATUS_MESSAGE_ID         0xFD
#define RESPONSE_MESSAGE_ID                 0xFC
#define WHO_AM_I_MESSAGE_ID                 0xFB
#define JOB_COMPLETE_MESSAGE_ID             0xFA
#define JOB_CANCELLED_MESSAGE_ID            0xF9

// command_types
#define SEND_JOB                            0xEF
#define SEND_JOB_WITH_RAMPING               0xEE
#define SEND_JOB_ALL_VARIABLES              0xED
#define SEND_JOB_ALL_VARIABLES_WITH_RAMPING 0xEC
#define PAUSE_JOB                           0xEB
#define RESUME_JOB                          0xEA
#define CANCEL_JOB                          0xE9
#define ENABLE_MOTOR                        0xE8
#define DISABLE_MOTOR                       0xE7
#define SLEEP_MOTOR                         0xE6
#define WAKE_MOTOR                          0xE5
#define RESET_MOTOR                         0xE4

// response_types
#define BAD_JOB_COMMAND_RESPONSE            0xDF
#define MOTOR_BUSY_RESPONSE                 0xDE
#define UNKNOWN_MOTOR_COMMAND_RESPONSE      0xDD
#define MOTOR_IN_FAULT_RESPONSE             0xDC
#define MOTOR_IN_SLEEP_RESPONSE             0xDB
#define MOTOR_PAUSED_RESPONSE               0xDA
#define MOTOR_DISABLED_RESPONSE             0xD9
#define NO_ACTIVE_JOB_RESPONSE              0xD8
#define JOB_ALREADY_PAUSED_RESPONSE         0xD7
#define JOB_ALREADY_RESUMED_RESPONSE        0xD6
#define MOTOR_ALREADY_ENABLED_RESPONSE      0xD5
#define MOTOR_ALREADY_DISABLED_RESPONSE     0xD4
#define MOTOR_ALREADY_SLEEPING_RESPONSE     0xD3
#define MOTOR_ALREADY_AWAKE_RESPONSE        0xD2
#define SLEEP_WITH_ACTIVE_JOB_RESPONSE      0xD1
#define WAKE_WITH_ACTIVE_JOB_RESPONSE       0xD0

// who_am_i
#define WHO_AM_I                            0xCF

// load_cell
#define NUMBER_OF_LOADCELLS                 1
#define LOAD_CELL_SAMPLE_INTERVAL           250

// load_cell_0
#define LOAD_CELL_0_ID                      0xB0
#define LOAD_CELL_0_IN_USE                  1

// motors
#define NUMBER_OF_MOTORS                    10
#define MINIMUM_PULSE_INTERVAL_uS           1000

// motor_0
#define MOTOR_0_ID                          0xA0
#define MOTOR_0_STEPS_PER_REV               200
#define MOTOR_0_IN_USE                      1

// motor_1
#define MOTOR_1_ID                          0xA1
#define MOTOR_1_STEPS_PER_REV               200
#define MOTOR_1_IN_USE                      1

// motor_2
#define MOTOR_2_ID                          0xA2
#define MOTOR_2_STEPS_PER_REV               200
#define MOTOR_2_IN_USE                      1

// motor_3
#define MOTOR_3_ID                          0xA3
#define MOTOR_3_STEPS_PER_REV               200
#define MOTOR_3_IN_USE                      1

// motor_4
#define MOTOR_4_ID                          0xA4
#define MOTOR_4_STEPS_PER_REV               200
#define MOTOR_4_IN_USE                      1

// motor_5
#define MOTOR_5_ID                          0xA5
#define MOTOR_5_STEPS_PER_REV               200
#define MOTOR_5_IN_USE                      1

// motor_6
#define MOTOR_6_ID                          0xA6
#define MOTOR_6_STEPS_PER_REV               200
#define MOTOR_6_IN_USE                      1

// motor_7
#define MOTOR_7_ID                          0xA7
#define MOTOR_7_STEPS_PER_REV               200
#define MOTOR_7_IN_USE                      1

// motor_8
#define MOTOR_8_ID                          0xA8
#define MOTOR_8_STEPS_PER_REV               200
#define MOTOR_8_IN_USE                      1

// motor_9
#define MOTOR_9_ID                          0xA9
#define MOTOR_9_STEPS_PER_REV               200
#define MOTOR_9_IN_USE                      1
