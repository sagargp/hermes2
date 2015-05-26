Battery *battery;
Motors *motors;

void setup()
{
  // attach servos
  // ...
  // set servos to default position
  // ...

  motors = new Motors();
  
  battery = new Battery();
  battery->disableCharger();

  Serial.begin(rate);
  Serial.flush();
}

void loop()
{
  if (battery->isOverCurrentCondition())
  {
    motors->stop();
  }

  // battery state machine
  // States: Drive motors; shut down; charge
  // Transitions: voltage low; plugged in; unplugged; voltage OK
  //

  if (battery->isUnderVoltage() && battery->isCharged())
  {
    motors->stop();
    battery->setCharged(false);
    battery->setStartVolts(battery->getVoltage());
  }

  if (battery->isPluggedIn() && !battery->isCharged())
  {
    battery->enableCharger();
    battery->charge();
  }
  else
  {
    // drive motors and stuff
  }
}
