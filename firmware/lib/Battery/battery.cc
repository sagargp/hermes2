#include "battery.h"
#include "configuration.h"

Battery::Battery()
{
}

int Battery::getVoltage()
{
  return analogRead(Pins::Battery);
}

int Battery::getLeftAmps()
{
  return analogRead(Pins::LeftMotorCurrent);
}

int Battery::getRightAmps()
{
  return analogRead(Pins::RightMotorCurrent);
}

bool Battery::isOverCurrentCondition()
{
  return getLeftAmps() > max_amps || getRightAmps() > max_amps;
}

bool Battery::isCharged()
{
  return m_is_charged;
}

void Battery::setCharged(bool charged)
{
  m_is_charged = charged;
}

void Battery::charge()
{
  int currVolts = getVoltage();

  if (currVolts > m_max_volts)
  {
    m_max_volts     = getVoltage();
    m_charger_timer = millis();
  }

  if (currVolts > peakVoltageThreshold)
  {
    if ((m_max_volts - currVolts) > 5 || (millis() - m_charger_timer) > charge_timeout)
    {
      // we're done charging
      setCharged(true);
      disableCharger();
    }
  }
}

bool Battery::isPluggedIn()
{
  if (getVoltage() - m_start_volts > convertVolts(1))
  {
    return true;
  }
  return false;
}

int Battery::convertVolts(int volts)
{
  return volts*67;
}

void Battery::setStartVolts(int volts)
{
  m_start_volts = volts;
}

void Battery::disableCharger()
{
  digitalWrite(Pins::Charger, 1);
}

void Battery::enableCharger()
{
  digitalWrite(Pins::Charger, 0);
}
