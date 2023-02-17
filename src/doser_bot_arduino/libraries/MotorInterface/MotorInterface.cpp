#include "MotorInterface.h"

MotorInterface::MotorInterface()
{

}

void MotorInterface::Initialise(int i2c_address, int pin, uint8_t bank)
{
    _i2c_addr = i2c_address;
    _step_pin = pin;

    if(bank == 'A')
    {
        _direction_register = MCP23017_IODIRA;
        _io_register = MCP23017_GPIOA;
        _pull_up_register = MCP23017_GPPUA;

    } else if(bank == 'B')
        {
            _direction_register = MCP23017_IODIRB;
            _io_register = MCP23017_GPIOB;
            _pull_up_register = MCP23017_GPPUB;
        }

    _output_state = false;
    _fault_check_interval = FAULT_CHECK_INTERVAL;
    _status_interval = STATUS_INTERVAL;
    _last_fault_check_micros = micros();
    _last_status_micros = micros();

    status_variables.direction = false;
    status_variables.running = false;
    status_variables.fault = false;
    status_variables.enabled = false;
    status_variables.paused = false;
    status_variables.sleep = false;
    status_variables.use_ramping = false;
    status_variables.running = false;
    status_variables.microstep = 1;
    status_variables.job_id = 0;
    status_variables.ramp_up_stop = 0;
    status_variables.ramp_down_start = 0;
    status_variables.pulses_remaining = 0;
    status_variables.pulse_interval = DEFAULT_PULSE_INTERVAL;
    status_variables.ramp_pulse_interval = DEFAULT_PULSE_INTERVAL;
    status_variables.pulse_on_period = DEFAULT_PULSE_ON_PERIOD;
    status_variables.ramp_up_interval = DEFAULT_PULSE_INTERVAL;
    status_variables.ramp_down_interval = DEFAULT_PULSE_INTERVAL;
    status_variables.ramp_interval_step = 0;

    command_variables.direction = false;
    command_variables.use_ramping = false;
    command_variables.microstep = 1;
    command_variables.job_id = 0;
    command_variables.ramping_steps = 0;
    command_variables.pulses = 0;
    command_variables.pulse_interval = DEFAULT_PULSE_INTERVAL;
    command_variables.pulse_on_period = DEFAULT_PULSE_ON_PERIOD;

    pinMode(_step_pin, OUTPUT);
    digitalWrite(_step_pin, LOW);

    WriteDirectionRegister(IO_DIRECTION);
    WriteIORegister(IO_STATE);
}

void MotorInterface::Enable()
{
    //Serial.println("enabled");
    status_variables.enabled = true;
    ReadIORegister();

    if(bitRead(_current_io, ENABLE_BIT))
    {
        bitClear(_current_io, ENABLE_BIT);
        WriteIORegister(_current_io);
    }
}

void MotorInterface::Disable()
{
    //Serial.println("disabled");
    status_variables.enabled = false;
    ReadIORegister();

    if(!bitRead(_current_io, ENABLE_BIT))
    {
        bitSet(_current_io, ENABLE_BIT);
        WriteIORegister(_current_io);
    }
}

void MotorInterface::Wake()
{
    //Serial.println("wake");
    status_variables.sleep = false;
    ReadIORegister();

    if(!bitRead(_current_io, SLEEP_BIT))
    {
        bitSet(_current_io, SLEEP_BIT);
        WriteIORegister(_current_io);

    }
}

void MotorInterface::Sleep()
{
    //Serial.println("sleep");
    status_variables.sleep = true;
    ReadIORegister();

    if(bitRead(_current_io, SLEEP_BIT))
    {
        bitClear(_current_io, SLEEP_BIT);
        WriteIORegister(_current_io);
    }
}

void MotorInterface::Reset()
{
    ReadIORegister();
    bitClear(_current_io, RESET_BIT);
    WriteIORegister(_current_io);
    delay(1);
    bitSet(_current_io, RESET_BIT);
    WriteIORegister(_current_io);
}

boolean MotorInterface::FaultStatus()
{
    ReadIORegister();
    return !bitRead(_current_io, FAULT_BIT);
}

uint8_t MotorInterface::DecodeMicroStep(uint8_t start_value)
{
    switch(status_variables.microstep)
    {
        case 1:
            bitClear(start_value, M0_BIT);
            bitClear(start_value, M1_BIT);
            bitClear(start_value, M2_BIT);
            break;

        case 2:
            bitSet(start_value, M0_BIT);
            bitClear(start_value, M1_BIT);
            bitClear(start_value, M2_BIT);
            break;

        case 4:
            bitClear(start_value, M0_BIT);
            bitSet(start_value, M1_BIT);
            bitClear(start_value, M2_BIT);
            break;

        case 8:
            bitSet(start_value, M0_BIT);
            bitSet(start_value, M1_BIT);
            bitClear(start_value, M2_BIT);
            break;

        case 16:
            bitClear(start_value, M0_BIT);
            bitClear(start_value, M1_BIT);
            bitSet(start_value, M2_BIT);
            break;

        case 32:
            bitSet(start_value, M0_BIT);
            bitClear(start_value, M1_BIT);
            bitSet(start_value, M2_BIT);
            break;

        default:
            bitClear(start_value, M0_BIT);
            bitClear(start_value, M1_BIT);
            bitClear(start_value, M2_BIT);
            status_variables.microstep = 1;
            break;
    }
    return start_value;
}

void MotorInterface::StartJob()
{
    ReadIORegister();

    if(!bitRead(_current_io, FAULT_BIT))
    {
        status_variables.fault = true;
        status_variables.running = false;
        Sleep();
        Disable();

    } else
        {
            status_variables.fault = false;
            status_variables.running = true;
            status_variables.direction = command_variables.direction;
            status_variables.use_ramping = command_variables.use_ramping;
            status_variables.microstep = command_variables.microstep;
            status_variables.job_id = command_variables.job_id;
            status_variables.paused = false;

            _output_state = false;
            _last_micros = 0;
            _last_pulse_on_micros = 0;
            _last_pulse_off_micros = 0;

            bitSet(_current_io, SLEEP_BIT);
            bitSet(_current_io, RESET_BIT);
            bitClear(_current_io, ENABLE_BIT);
            bitWrite(_current_io, DIRECTION_BIT, status_variables.direction);

            _current_io = DecodeMicroStep(_current_io);

            WriteIORegister(_current_io);

            status_variables.pulse_interval = (command_variables.pulse_interval > MINIMUM_PULSE_INTERVAL && command_variables.pulse_interval < MAXIMUM_PULSE_INTERVAL) ? command_variables.pulse_interval : DEFAULT_PULSE_INTERVAL;
            status_variables.pulse_on_period = (command_variables.pulse_on_period < command_variables.pulse_interval) ? command_variables.pulse_on_period : (long)(command_variables.pulse_interval/2);
            status_variables.pulses_remaining = command_variables.pulses;

            if(status_variables.use_ramping)
            {
                if(2*command_variables.ramping_steps < status_variables.pulses_remaining)
                {
                    status_variables.ramp_up_stop = status_variables.pulses_remaining - command_variables.ramping_steps;
                    status_variables.ramp_down_start = command_variables.ramping_steps;

                } else
                    {
                        status_variables.ramp_up_stop = (long)(status_variables.pulses_remaining/2);
                        status_variables.ramp_down_start = status_variables.ramp_up_stop-1;
                    }

                status_variables.ramp_up_interval = status_variables.pulse_interval * 3;
                status_variables.ramp_pulse_interval = status_variables.ramp_up_interval;
                status_variables.ramp_down_interval = status_variables.pulse_interval;
                status_variables.ramp_interval_step = (long)((status_variables.ramp_up_interval - status_variables.ramp_down_interval) / command_variables.ramping_steps);

            } else
                {
                    status_variables.ramp_up_stop = 0;
                    status_variables.ramp_down_start = 0;
                    status_variables.ramp_up_interval = 0;
                    status_variables.ramp_down_interval = 0;
                    status_variables.ramp_interval_step = 0;
                    status_variables.ramp_pulse_interval = 0;
                }

            digitalWrite(_step_pin, LOW);
        }
}

void MotorInterface::PauseJob()
{
    status_variables.paused = true;
}

void MotorInterface::ResumeJob()
{
    status_variables.paused = false;
    _last_pulse_on_micros = 0;
    _last_pulse_off_micros = 0;
}

void MotorInterface::CancelJob()
{
    status_variables.running = false;
    _output_state = false;
    status_variables.pulses_remaining = 0;
    digitalWrite(_step_pin, LOW);
}

void MotorInterface::ResetJobId()
{
    status_variables.job_id = 0;
}

boolean MotorInterface::Update(long micros_now)
{
    boolean job_done = false;
    if(status_variables.enabled)
    {
        if(status_variables.running && !status_variables.paused && !status_variables.fault)
        {
            if(status_variables.pulses_remaining > 0)
            {
                if(status_variables.use_ramping)
                {
                    if(!_output_state)
                    {
                        if(abs(micros_now - _last_pulse_on_micros) >= status_variables.ramp_pulse_interval)
                        {
                            _last_pulse_on_micros += status_variables.pulse_interval;
                            _last_pulse_off_micros = micros_now;
                            digitalWrite(_step_pin, HIGH);
                            _output_state = true;

                            if(status_variables.pulses_remaining > status_variables.ramp_up_stop)
                            {
                                status_variables.ramp_up_interval -= status_variables.ramp_interval_step;
                                status_variables.ramp_pulse_interval = status_variables.ramp_up_interval;

                            } else if(status_variables.pulses_remaining < status_variables.ramp_down_start)
                                {
                                    status_variables.ramp_down_interval += status_variables.ramp_interval_step;
                                    status_variables.ramp_pulse_interval = status_variables.ramp_down_interval;

                                } else
                                    {
                                        status_variables.ramp_pulse_interval = status_variables.pulse_interval;
                                    }
                        }
                    } else
                        {
                            if(abs(micros_now - _last_pulse_off_micros) >= status_variables.pulse_on_period)
                            {
                                digitalWrite(_step_pin, LOW);
                                _output_state = false;
                                status_variables.pulses_remaining--;
                            }
                        }

                } else
                    {
                        if(!_output_state)
                        {
                            if(abs(micros_now - _last_pulse_on_micros) >= status_variables.pulse_interval)
                            {
                                _last_pulse_on_micros = micros_now;
                                _last_pulse_off_micros = micros_now;
                                digitalWrite(_step_pin, HIGH);
                                _output_state = true;
                            }

                        } else
                            {
                                if(abs(micros_now - _last_pulse_off_micros) >= status_variables.pulse_on_period)
                                {
                                    digitalWrite(_step_pin, LOW);
                                    _output_state = false;
                                    status_variables.pulses_remaining--;
                                }
                            }
                    }
            } else
                {
                    status_variables.running = false;
                    job_done = true;
                }
        }
    }

    _last_micros = micros_now;

    if(abs(micros_now - _last_fault_check_micros) >= _fault_check_interval)
    {
        _last_fault_check_micros = micros_now;

        if(FaultStatus())
        {
             status_variables.fault = true;
        } else
            {
                status_variables.fault = false;
            }
    }

    if(abs(micros_now - _last_status_micros) >= _status_interval)
    {
        _last_status_micros = micros_now;
        bitWrite(status_byte, DIRECTION_BIT, status_variables.direction); // Bit 0
        bitWrite(status_byte, FAULT_BIT, status_variables.fault); // Bit 1
        bitWrite(status_byte, PAUSED_BIT, status_variables.paused); // Bit 2
        bitWrite(status_byte, 3, status_variables.use_ramping); // Bit 3
        // bitWrite(status_byte, 4, ); // Bit 4
        bitWrite(status_byte, ENABLE_BIT, status_variables.enabled); // Bit 5
        bitWrite(status_byte, RUNNING_BIT, status_variables.running); // Bit 6
        bitWrite(status_byte, SLEEP_BIT, status_variables.sleep); // Bit 7
    }

    return job_done;
}

void MotorInterface::ReadIORegister()
{
  Wire.beginTransmission(_i2c_addr);
  Wire.write(_io_register);
  Wire.endTransmission();
  Wire.requestFrom(_i2c_addr, 1);
  _current_io = Wire.read();
}

void MotorInterface::WriteIORegister(uint8_t val)
{
  Wire.beginTransmission(_i2c_addr);
  Wire.write(_io_register);
  Wire.write(val);
  Wire.endTransmission();
}

void MotorInterface::WriteDirectionRegister(uint8_t val)
{
  Wire.beginTransmission(_i2c_addr);
  Wire.write(_direction_register);
  Wire.write(val);
  Wire.endTransmission();
}

void MotorInterface::WritePullUpRegister(uint8_t val)
{
  Wire.beginTransmission(_i2c_addr);
  Wire.write(_direction_register);
  Wire.write(_pull_up_register);
  Wire.endTransmission();
}

