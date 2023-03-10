#ifndef MotorInterface_h
#define MotorInterface_h

#include "Arduino.h"
#include <Wire.h>

// Bits 0 = Direction (out), 1 = Fault (in), 2,3 & 4 = M0, M1 & M2 (out), 5 = Enable (out), 6 = Reset (out), 7 = Sleep (out)
#define DIRECTION_BIT 0
#define FAULT_BIT 1
#define M0_BIT 2
#define M1_BIT 3
#define M2_BIT 4
#define ENABLE_BIT 5
#define RESET_BIT 6
#define RUNNING_BIT 6
#define SLEEP_BIT 7
#define PAUSED_BIT 2

#define IO_DIRECTION 0x02
#define IO_STATE 0xC0

#define DEFAULT_PULSE_ON_PERIOD 500
#define DEFAULT_PULSE_INTERVAL  1000
#define MINIMUM_PULSE_INTERVAL  500
#define MAXIMUM_PULSE_INTERVAL  1000000
#define FAULT_CHECK_INTERVAL    100000
#define STATUS_INTERVAL         10000

//Address of MCP23017 IO Expander, 8 addresses available
#define MCP23017_ADDR_0	     0x20
#define MCP23017_ADDR_1	     0x21
#define MCP23017_ADDR_2	     0x22
#define MCP23017_ADDR_3	     0x23
#define MCP23017_ADDR_4	     0x24
#define MCP23017_ADDR_5	     0x25
#define MCP23017_ADDR_6	     0x26
#define MCP23017_ADDR_7	     0x27

#define MCP23017_IODIRA      0x00
#define MCP23017_GPIOA       0x12
#define MCP23017_GPPUA       0x0C

#define MCP23017_IODIRB      0x01
#define MCP23017_GPIOB       0x13
#define MCP23017_GPPUB       0x0D


struct motor_command_struct {

  boolean direction;
  boolean use_ramping;
  uint8_t microstep;
  uint8_t job_id;
  long ramping_steps;
  long pulse_interval;
  long pulses;
  long pulse_on_period;
};

struct motor_status_struct {

  boolean running;
  boolean fault;
  boolean direction;
  boolean enabled;
  boolean sleep;
  boolean paused;
  boolean use_ramping;
  uint8_t microstep;
  uint8_t job_id;
  long ramp_up_stop;
  long ramp_down_start;
  long ramp_up_interval;
  long ramp_down_interval;
  long ramp_interval_step;
  long ramp_pulse_interval;
  long pulse_interval;
  long pulse_on_period;
  long pulses_remaining;
};

class MotorInterface
{
public:
        MotorInterface();
        void Initialise(int, int, uint8_t);
        void Enable();
        void Disable();
        void Wake();
        void Sleep();
        void Reset();
        boolean FaultStatus();
        void StartJob();
        void PauseJob();
        void ResumeJob();
        void CancelJob();
        void ResetJobId();
        boolean Update(long);

        motor_command_struct command_variables;
        motor_status_struct status_variables;
        uint8_t status_byte;

private:
        uint8_t DecodeMicroStep(uint8_t);
        void ReadIORegister();
        void WriteIORegister(uint8_t);
        void WriteDirectionRegister(uint8_t);
        void WritePullUpRegister(uint8_t);

        boolean _output_state;
        uint8_t  _current_io, _direction_register, _io_register, _pull_up_register;
        int _i2c_addr, _step_pin;
        long _last_fault_check_micros, _fault_check_interval, _status_interval, _last_status_micros;
        long _last_pulse_on_micros, _last_pulse_off_micros, _last_micros;
};

#endif
