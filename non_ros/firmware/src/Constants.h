// Communications
#define CMODE_RC             0
#define CMODE_SERIAL         1
#define CMODE_I2C            2

#define CMODE     CMODE_SERIAL // Sets communication mode: 0=RC    1=Serial    2=I2C
#define BRATE             9600 // Baud rate for serial communications
#define I2C_ADDR          0x04 // Address for I2C

// Communication command constants. These are computed as (A << 8) + B.
#define COMMAND_CH       17224 // Change mode (I2C <--> Serial)
#define COMMAND_VO       22095 // Get voltage
#define COMMAND_FL       17996 // Flush buffers
#define COMMAND_AN       16718 // Read analog pins
#define COMMAND_SV       21334 // Set servo positions
#define COMMAND_HB       18498 // Set motors

// Battery settings
#define BATVOLT            487 // This is the nominal battery voltage reading. Peak charge can only occur above this voltage.
#define LOWVOLT            410 // This is the voltage at which the speed controller goes into recharge mode.
#define CHARGETIMEOUT   300000 // If the battery voltage does not change in this number of milliseconds then stop charging.

// H-bridge settings
#define LEFTMAXAMPS        800 // set overload current for left motor 
#define RIGHTMAXAMPS       800 // set overload current for right motor 
#define OVERLOADTIME       100 // time in mS before motor is re-enabled after overload occurs

// Servo settings
#define DSERVO0           1500 // default position for servo0 on "power up" - 1500uS is center position on most servos
#define DSERVO1           1500 // default position for servo1 on "power up" - 1500uS is center position on most servos
#define DSERVO2           1500 // default position for servo2 on "power up" - 1500uS is center position on most servos
#define DSERVO3           1500 // default position for servo3 on "power up" - 1500uS is center position on most servos
#define DSERVO4           1500 // default position for servo4 on "power up" - 1500uS is center position on most servos
#define DSERVO5           1500 // default position for servo5 on "power up" - 1500uS is center position on most servos
#define DSERVO6           1500 // default position for servo6 on "power up" - 1500uS is center position on most servos

// RC mode options (unused)
#define MIX                  1 // Set to 1 if L/R and F/R signals from RC need to be mixed
#define LEFTCENTER        1500 // when RC inputs are centered then input should be 1.5mS
#define RIGHTCENTER       1500 // when RC inputs are centered then input should be 1.5mS
#define RCDEADBAND          35 // inputs do not have to be perfectly centered to stop motors
#define SCALE               12 // scale factor for RC signal to PWM
