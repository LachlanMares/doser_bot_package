
void initialiseSerial()
{
  last_motor_status_update = micros();
  last_motor_pulses_update = micros();
  serialport.setInitial(1, BAUD_RATE, SERIAL_TIMEOUT_MS);
}

void updateSerial()
{
  int bytes_read = serialport.update(&serial_buffer[0]);

  if(bytes_read > 0)
  {
    switch(serial_buffer[0])
    {
      case WHO_AM_I:
        {
          uint8_t who_buffer[10] = {WHO_AM_I_MESSAGE_ID, 0x44, 0x6F, 0x73, 0x65, 0x72, 0x2D, 0x42, 0x6F, 0x74};
          serialport.sendMessage(&who_buffer[0], 10);
          delay(1000);
          enable_status_message = true;
          Serial.println("enabled");
        }
        break;

      case MOTOR_0_ID:
        processCommandMessage(&m0, MOTOR_0_ID, bytes_read);
        break;

      case MOTOR_1_ID:
        processCommandMessage(&m1, MOTOR_1_ID, bytes_read);
        break;

      case MOTOR_2_ID:
        processCommandMessage(&m2, MOTOR_2_ID, bytes_read);
        break;

      case MOTOR_3_ID:
        processCommandMessage(&m3, MOTOR_3_ID, bytes_read);
        break;

      case MOTOR_4_ID:
        processCommandMessage(&m4, MOTOR_4_ID, bytes_read);
        break;

      case MOTOR_5_ID:
        processCommandMessage(&m5, MOTOR_5_ID, bytes_read);
        break;

      case MOTOR_6_ID:
        processCommandMessage(&m6, MOTOR_6_ID, bytes_read);
        break;

      case MOTOR_7_ID:
        processCommandMessage(&m7, MOTOR_7_ID, bytes_read);
        break;

      case MOTOR_8_ID:
        processCommandMessage(&m8, MOTOR_8_ID, bytes_read);
        break;

      case MOTOR_9_ID:
        processCommandMessage(&m9, MOTOR_9_ID, bytes_read);
        break;
    }
  }

  if(enable_status_message)
  {
    statusUpdate(millis());
  }
}

void statusUpdate(long millis_now)
{
  // Motor status
  if(abs(millis_now - last_motor_status_update) >= MOTOR_STATUS_UPDATE_INTERVAL)
  {
    last_motor_status_update = millis_now;
    uint8_t motor_status_buffer[21];
    motor_status_buffer[0] = MOTOR_STATUS_MESSAGE_ID;
    
    motor_status_buffer[1] = m0.status_byte;
    motor_status_buffer[2] = m0.status_variables.job_id;
    
    motor_status_buffer[3] = m1.status_byte;
    motor_status_buffer[4] = m1.status_variables.job_id;   

    motor_status_buffer[5] = m2.status_byte;
    motor_status_buffer[6] = m2.status_variables.job_id;
    
    motor_status_buffer[7] = m3.status_byte;
    motor_status_buffer[8] = m3.status_variables.job_id;   

    motor_status_buffer[9] = m4.status_byte;
    motor_status_buffer[10] = m4.status_variables.job_id;
    
    motor_status_buffer[11] = m5.status_byte;
    motor_status_buffer[12] = m5.status_variables.job_id;   

    motor_status_buffer[13] = m6.status_byte;
    motor_status_buffer[14] = m6.status_variables.job_id;
    
    motor_status_buffer[15] = m7.status_byte;
    motor_status_buffer[16] = m7.status_variables.job_id;  

    motor_status_buffer[17] = m8.status_byte;
    motor_status_buffer[18] = m8.status_variables.job_id;
    
    motor_status_buffer[19] = m9.status_byte;
    motor_status_buffer[20] = m9.status_variables.job_id;  

    serialport.sendMessage(&motor_status_buffer[0], 21);
  }
  
  // Motor pulses status
  if(abs(millis_now - last_motor_pulses_update) >= MOTOR_PULSES_UPDATE_INTERVAL)
  {
    last_motor_pulses_update = millis_now;
    uint8_t motor_pulses_buffer[41];
    motor_pulses_buffer[0] = MOTOR_PULSES_MESSAGE_ID;

    bytesFromLong(m0.status_variables.pulses_remaining, &motor_pulses_buffer[1]);
    bytesFromLong(m1.status_variables.pulses_remaining, &motor_pulses_buffer[5]);
    bytesFromLong(m2.status_variables.pulses_remaining, &motor_pulses_buffer[9]);
    bytesFromLong(m3.status_variables.pulses_remaining, &motor_pulses_buffer[13]);
    bytesFromLong(m4.status_variables.pulses_remaining, &motor_pulses_buffer[17]);
    bytesFromLong(m5.status_variables.pulses_remaining, &motor_pulses_buffer[21]);
    bytesFromLong(m6.status_variables.pulses_remaining, &motor_pulses_buffer[25]);
    bytesFromLong(m7.status_variables.pulses_remaining, &motor_pulses_buffer[29]);
    bytesFromLong(m8.status_variables.pulses_remaining, &motor_pulses_buffer[33]);
    bytesFromLong(m9.status_variables.pulses_remaining, &motor_pulses_buffer[37]);
    
    serialport.sendMessage(&motor_pulses_buffer[0], 41);
  }
  
  // Load cell status
  if(abs(millis_now - last_load_cell_update) >= LOAD_CELL_UPDATE_INTERVAL)
  {
    last_load_cell_update = millis_now;
    uint8_t load_cell_update_buffer[5] = {LOAD_CELL_STATUS_MESSAGE_ID, 0x00, 0x00, 0x00, 0x00};
    bytesFromLong(load_cell_raw, &load_cell_update_buffer[1]);
    serialport.sendMessage(&load_cell_update_buffer[0], 5);
  }
}

void processCommandMessage(MotorInterface* motor_ptr, uint8_t motor_num, int bytes_read)
{
  uint8_t response_buffer[5] = {RESPONSE_MESSAGE_ID, motor_num, serial_buffer[1], 0x00, ACK};
  switch(serial_buffer[1])
  {
    case SEND_JOB:

      if(motor_ptr->status_variables.fault)
      {
        response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;
        response_buffer[4] = NAK;

      } else if (!motor_ptr->status_variables.enabled)
        {
          response_buffer[3] = MOTOR_DISABLED_RESPONSE;
          response_buffer[4] = NAK;

        } else if(motor_ptr->status_variables.running)
          {
            response_buffer[3] = MOTOR_BUSY_RESPONSE;
            response_buffer[4] = NAK;

          } else if (motor_ptr->status_variables.sleep)
            {
              response_buffer[3] = MOTOR_IN_SLEEP_RESPONSE;
              response_buffer[4] = NAK;

            } else if (bytes_read == 9)
              {
                motor_ptr->command_variables.use_ramping = false;
                motor_ptr->command_variables.direction = (serial_buffer[2] > 0) ? true : false;
                motor_ptr->command_variables.microstep = serial_buffer[3];
                motor_ptr->command_variables.job_id = serial_buffer[4];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[5]);
                motor_ptr->command_variables.pulse_interval = 0;
                motor_ptr->command_variables.pulse_on_period = 0;
                motor_ptr->command_variables.ramping_steps = 0;
                motor_ptr->StartJob();

              } else
                {
                  response_buffer[3] = BAD_JOB_COMMAND_RESPONSE;
                  response_buffer[4] = NAK;
                }
      break;

    case SEND_JOB_WITH_RAMPING:

      if(motor_ptr->status_variables.fault)
      {
        response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;
        response_buffer[4] = NAK;

      } else if (!motor_ptr->status_variables.enabled)
        {
          response_buffer[3] = MOTOR_DISABLED_RESPONSE;
          response_buffer[4] = NAK;

        } else if(motor_ptr->status_variables.running)
          {
            response_buffer[3] = MOTOR_BUSY_RESPONSE;
            response_buffer[4] = NAK;

          } else if (motor_ptr->status_variables.sleep)
            {
              response_buffer[3] = MOTOR_IN_SLEEP_RESPONSE;
              response_buffer[4] = NAK;

            } else if (bytes_read == 13)
              {
                motor_ptr->command_variables.use_ramping = true;
                motor_ptr->command_variables.direction = (serial_buffer[2] > 0) ? true : false;
                motor_ptr->command_variables.microstep = serial_buffer[3];
                motor_ptr->command_variables.job_id = serial_buffer[4];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[5]);
                motor_ptr->command_variables.ramping_steps = longFromBytes(&serial_buffer[9]);
                motor_ptr->command_variables.pulse_interval = 0;
                motor_ptr->command_variables.pulse_on_period = 0;
                motor_ptr->StartJob();

              } else
                {
                  response_buffer[3] = BAD_JOB_COMMAND_RESPONSE;
                  response_buffer[4] = NAK;
                }
      break;

    case SEND_JOB_ALL_VARIABLES:

      if(motor_ptr->status_variables.fault)
      {
        response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;
        response_buffer[4] = NAK;

      } else if (!motor_ptr->status_variables.enabled)
        {
          response_buffer[3] = MOTOR_DISABLED_RESPONSE;
          response_buffer[4] = NAK;

        } else if(motor_ptr->status_variables.running)
          {
            response_buffer[3] = MOTOR_BUSY_RESPONSE;
            response_buffer[4] = NAK;

          } else if (motor_ptr->status_variables.sleep)
            {
              response_buffer[3] = MOTOR_IN_SLEEP_RESPONSE;
              response_buffer[4] = NAK;

            } else if (bytes_read == 17)
              {
                motor_ptr->command_variables.use_ramping = false;
                motor_ptr->command_variables.direction = (serial_buffer[2] > 0) ? true : false;
                motor_ptr->command_variables.microstep = serial_buffer[3];
                motor_ptr->command_variables.job_id = serial_buffer[4];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[5]);
                motor_ptr->command_variables.pulse_interval = longFromBytes(&serial_buffer[9]);
                motor_ptr->command_variables.pulse_on_period = longFromBytes(&serial_buffer[13]);
                motor_ptr->StartJob();

              } else
                {
                  response_buffer[3] = BAD_JOB_COMMAND_RESPONSE;
                  response_buffer[4] = NAK;
                }
      break;

    case SEND_JOB_ALL_VARIABLES_WITH_RAMPING:

      if(motor_ptr->status_variables.fault)
      {
        response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;
        response_buffer[4] = NAK;

      } else if (!motor_ptr->status_variables.enabled)
        {
          response_buffer[3] = MOTOR_DISABLED_RESPONSE;
          response_buffer[4] = NAK;

        } else if(motor_ptr->status_variables.running)
          {
            response_buffer[3] = MOTOR_BUSY_RESPONSE;
            response_buffer[4] = NAK;

          } else if (motor_ptr->status_variables.sleep)
            {
              response_buffer[3] = MOTOR_IN_SLEEP_RESPONSE;
              response_buffer[4] = NAK;

            } else if (bytes_read == 21)
              {
                motor_ptr->command_variables.use_ramping = true;
                motor_ptr->command_variables.direction = (serial_buffer[2] > 0) ? true : false;;
                motor_ptr->command_variables.microstep = serial_buffer[3];
                motor_ptr->command_variables.job_id = serial_buffer[4];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[5]);
                motor_ptr->command_variables.pulse_interval = longFromBytes(&serial_buffer[9]);
                motor_ptr->command_variables.pulse_on_period = longFromBytes(&serial_buffer[13]);
                motor_ptr->command_variables.ramping_steps = longFromBytes(&serial_buffer[17]);
                motor_ptr->StartJob();

              } else
                {
                  response_buffer[3] = BAD_JOB_COMMAND_RESPONSE;
                  response_buffer[4] = NAK;
                }
      break;

    case PAUSE_JOB:

        if(motor_ptr->status_variables.fault)
        {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;
          if(motor_ptr->status_variables.running)
          {
            motor_ptr->PauseJob();
          }

        } else if (motor_ptr->status_variables.enabled)
          {
            if(motor_ptr->status_variables.running)
            {
              if (motor_ptr->status_variables.paused)
              {
                response_buffer[3] = JOB_ALREADY_PAUSED_RESPONSE;
                response_buffer[4] = NAK;

              } else
                {
                  motor_ptr->PauseJob();
                }
            } else
              {
                response_buffer[3] = NO_ACTIVE_JOB_RESPONSE;
                response_buffer[4] = NAK;
              }
          } else
            {
              response_buffer[3] = MOTOR_DISABLED_RESPONSE;
              response_buffer[4] = NAK;
            }
      break;

    case RESUME_JOB:

        if(motor_ptr->status_variables.fault)
        {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;
          response_buffer[4] = NAK;

        } else if (motor_ptr->status_variables.enabled)
          {
            if(motor_ptr->status_variables.running)
            {
              if(motor_ptr->status_variables.paused)
              {
                motor_ptr->ResumeJob();

              } else
                {
                  response_buffer[3] = JOB_ALREADY_RESUMED_RESPONSE;
                  response_buffer[4] = NAK;
                }
            } else
              {
                response_buffer[3] = NO_ACTIVE_JOB_RESPONSE;
                response_buffer[4] = NAK;
              }
          } else
            {
              response_buffer[3] = MOTOR_DISABLED_RESPONSE;
              response_buffer[4] = NAK;
            }

      break;

    case CANCEL_JOB:

        if (motor_ptr->status_variables.enabled)
        {
          if(motor_ptr->status_variables.running)
          {
            motor_ptr->CancelJob();
            uint8_t job_cancelled_buffer[3] = {JOB_CANCELLED_MESSAGE_ID, motor_num, motor_ptr->status_variables.job_id};
            serialport.sendMessage(&job_cancelled_buffer[0], 3);
            motor_ptr->ResetJobId();
          } else
              {
                response_buffer[3] = NO_ACTIVE_JOB_RESPONSE;
                response_buffer[4] = NAK;
              }
          } else
            {
              response_buffer[3] = MOTOR_DISABLED_RESPONSE;
              response_buffer[4] = NAK;
            }

      break;

    case ENABLE_MOTOR:

        if(motor_ptr->status_variables.fault)
        {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;
          response_buffer[4] = NAK;

        } else if (motor_ptr->status_variables.enabled)
          {
            response_buffer[3] = MOTOR_ALREADY_ENABLED_RESPONSE;
            response_buffer[4] = NAK;

          } else
            {
              motor_ptr->Enable();
            }

      break;

    case DISABLE_MOTOR:

        if(motor_ptr->status_variables.fault)
        {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;
          response_buffer[4] = NAK;

        } else if (motor_ptr->status_variables.enabled)
          {
            motor_ptr->Disable();

          } else
            {
              response_buffer[3] = MOTOR_ALREADY_DISABLED_RESPONSE;
              response_buffer[4] = NAK;
            }

      break;

    case SLEEP_MOTOR:

        if(motor_ptr->status_variables.fault)
        {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;
          if(!motor_ptr->status_variables.sleep)
          {
            motor_ptr->Sleep();
          }

        } else if (motor_ptr->status_variables.sleep)
          {
            response_buffer[3] = MOTOR_ALREADY_SLEEPING_RESPONSE;
            response_buffer[4] = NAK;

          } else
            {
              if(motor_ptr->status_variables.enabled && motor_ptr->status_variables.running)
              {
                motor_ptr->PauseJob();
                response_buffer[3] = SLEEP_WITH_ACTIVE_JOB_RESPONSE;
              }
              motor_ptr->Sleep();
            }

      break;

    case WAKE_MOTOR:

        if(motor_ptr->status_variables.fault)
        {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;
          response_buffer[4] = NAK;

        } else if(motor_ptr->status_variables.sleep)
          {
            motor_ptr->Wake();
            if(motor_ptr->status_variables.enabled && motor_ptr->status_variables.running && motor_ptr->status_variables.paused)
            {
              response_buffer[3] = WAKE_WITH_ACTIVE_JOB_RESPONSE;
              motor_ptr->ResumeJob();
            }

          } else
            {
              response_buffer[3] = MOTOR_ALREADY_AWAKE_RESPONSE;
              response_buffer[4] = NAK;
            }

      break;

    case RESET_MOTOR:

      {
        boolean was_enabled = false;
        boolean was_awake = false;
        boolean was_running = false;

        if(motor_ptr->status_variables.enabled)
        {
          was_enabled = true;

          if(motor_ptr->status_variables.running)
          {
            motor_ptr->PauseJob();
            was_running = true;
          }

          if(!motor_ptr->status_variables.sleep)
          {
            motor_ptr->Sleep();
            was_awake = true;
          }
        }

        motor_ptr->Reset();

        if(was_enabled)
        {
          motor_ptr->Enable();
        }

        if(was_awake)
        {
          motor_ptr->Wake();
        }

        if(was_running)
        {
          motor_ptr->ResumeJob();
        }
      }

      break;

    default:
      response_buffer[3] = UNKNOWN_MOTOR_COMMAND_RESPONSE;
      response_buffer[4] = NAK;
      break;
  }

  serialport.sendMessage(&response_buffer[0], 5);
}
