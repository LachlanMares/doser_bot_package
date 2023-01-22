void initialiseLoadcell()
{
  last_load_cell_update = millis();
  last_load_cell_sample = millis();
  scale.begin(load_cell_data_pin, load_cell_clock_pin);
}

void updateLoadcell(long millis_now)
{
  if(abs(millis_now - last_load_cell_sample) >= LOAD_CELL_SAMPLE_INTERVAL)
  {
    if(scale.is_ready())
    {
      last_load_cell_update = millis_now;
      load_cell_raw = scale.read();
    }
  }  
}
