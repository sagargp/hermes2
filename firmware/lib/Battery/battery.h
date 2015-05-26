#ifndef __BATTERY_H__
#define __BATTERY_H__

class Battery
{
  private:
    /* Max amperage allowance of each motor */
    int m_max_amps; 

    /* Lowest allowable voltage before shutting down */
    int m_min_voltage_threshold;

    /* Is the battery fully charged? */
    bool m_is_charged;
    
  public:
    Battery();

    // Getters
    int getVoltage();

    int getLeftAmps();

    int getRightAmps();

    bool isOverCurrentCondition();

    bool isCharged();

    bool isPluggedIn();

    // Setters
    void setCharged(bool charged);

    void setStartVolts(int volts);

    void disableCharger();

    void enableCharger();

    void charge();

    int convertVolts(int volts);
};

#endif
