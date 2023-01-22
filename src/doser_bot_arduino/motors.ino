
void initialiseMotors()
{
  m0.Initialise(MCP23017_ADDR_0, m0_step_pin, 'A');
  m0.Enable();
  m0.Sleep();

  m1.Initialise(MCP23017_ADDR_1, m1_step_pin, 'A');
  m1.Enable();
  m1.Sleep();

  m2.Initialise(MCP23017_ADDR_2, m2_step_pin, 'A');
  m2.Enable();
  m2.Sleep();
  
  m3.Initialise(MCP23017_ADDR_3, m3_step_pin, 'A');
  m3.Enable();
  m3.Sleep();
    
  m4.Initialise(MCP23017_ADDR_4, m4_step_pin, 'A');
  m4.Enable();
  m4.Sleep();
  
  m5.Initialise(MCP23017_ADDR_4, m5_step_pin, 'B');
  m5.Enable();
  m5.Sleep();
  
  m6.Initialise(MCP23017_ADDR_3, m6_step_pin, 'B');
  m6.Enable();
  m6.Sleep();
  
  m7.Initialise(MCP23017_ADDR_2, m7_step_pin, 'B');
  m7.Enable();
  m7.Sleep();
  
  m8.Initialise(MCP23017_ADDR_1, m8_step_pin, 'B');
  m8.Enable();
  m8.Sleep();
  
  m9.Initialise(MCP23017_ADDR_0, m9_step_pin, 'B');
  m9.Enable();
  m9.Sleep();
}

void updateMotors()
{
  uint8_t job_complete_buffer[3] = {JOB_COMPLETE_MESSAGE_ID, 0x00, 0x00};
  
  if(m0.Update(micros()))
  {
    job_complete_buffer[1] = MOTOR_0_ID;
    job_complete_buffer[2] = m0.status_variables.job_id;
    serialport.sendMessage(&job_complete_buffer[0], 3);
    m0.ResetJobId();
  }
  
  if(m1.Update(micros()))
  {
    job_complete_buffer[1] = MOTOR_1_ID;
    job_complete_buffer[2] = m1.status_variables.job_id;
    serialport.sendMessage(&job_complete_buffer[0], 3);
    m1.ResetJobId();
  }
  
  if(m2.Update(micros()))
  {
    job_complete_buffer[1] = MOTOR_2_ID;
    job_complete_buffer[2] = m2.status_variables.job_id;
    serialport.sendMessage(&job_complete_buffer[0], 3);
    m2.ResetJobId();
  }
  
  if(m3.Update(micros()))
  {
    job_complete_buffer[1] = MOTOR_3_ID;
    job_complete_buffer[2] = m3.status_variables.job_id;
    serialport.sendMessage(&job_complete_buffer[0], 3);
    m3.ResetJobId();
  }
  
  if(m4.Update(micros()))
  {
    job_complete_buffer[1] = MOTOR_4_ID;
    job_complete_buffer[2] = m4.status_variables.job_id;
    serialport.sendMessage(&job_complete_buffer[0], 3);
    m4.ResetJobId();
  }
  
  if(m5.Update(micros()))
  {
    job_complete_buffer[1] = MOTOR_5_ID;
    job_complete_buffer[2] = m5.status_variables.job_id;
    serialport.sendMessage(&job_complete_buffer[0], 3);
    m5.ResetJobId();
  }
  
  if(m6.Update(micros()))
  {
    job_complete_buffer[1] = MOTOR_6_ID;
    job_complete_buffer[2] = m6.status_variables.job_id;
    serialport.sendMessage(&job_complete_buffer[0], 3);
    m6.ResetJobId();
  }
  
  if(m7.Update(micros()))
  {
    job_complete_buffer[1] = MOTOR_7_ID;
    job_complete_buffer[2] = m7.status_variables.job_id;
    serialport.sendMessage(&job_complete_buffer[0], 3);
    m7.ResetJobId();
  }
  
  if(m8.Update(micros()))
  {
    job_complete_buffer[1] = MOTOR_8_ID;
    job_complete_buffer[2] = m8.status_variables.job_id;
    serialport.sendMessage(&job_complete_buffer[0], 3);
    m8.ResetJobId();
  }
  
  if(m9.Update(micros()))
  {
    job_complete_buffer[1] = MOTOR_9_ID;
    job_complete_buffer[2] = m9.status_variables.job_id;
    serialport.sendMessage(&job_complete_buffer[0], 3);
    m9.ResetJobId();
  }
}
