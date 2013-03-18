/*****************************************************************
Example code for the STMicro L6470 dSPIN stepper motor driver.
This code is public domain beerware/Sunny-D-ware. If you find it useful and
run into me someday, I'd appreciate a cold one.

12/12/2011- Mike Hord, SparkFun Electronics

The breakout board for the dSPIN chip has 7 data lines:
BSYN- this line is LOW when the chip is busy; busy generally means things
   like executing a move command.
STBY- drag low to reset the device to default conditions. Also should be
   performed after power up to ensure a known-good initial state.
FLGN- when the dSPIN raises a flag it usually means an error has occurred
STCK- used as a step clock input; the direction (and activation of this input
   is done by setting registers on the chip.
SDI- SPI data FROM the uC TO the dSPIN
SDO- SPI data TO the uC FROM the dSPIN
CSN- active-low slave select for the SPI bus
CK- data clock for the SPI bus

A note about connecting motors:
It's unclear from the datasheet what gets connected to which terminal.
Bridge one (terminals 01A and 01B) gets one coil, and bridge two gets
the other coil. For our mid-small stepper (ROB-9238), that translates to
  01A -> RED
  01B -> GREEN
  02A -> BLUE
  02B -> YELLOW

ERRATA: IMPORTANT!!! READ THIS!!!
There are several errors in the datasheet for the L6470:
- the internal oscillator is specified as 16MHz +/- 3%. Experimentally, it
   seems to be more like a 6% tolerance.
- when transitioning from one movement command to another, it may be necessary
   to include a dSPIN_SoftStop() between the two to ensure proper operation. For
   example, if dSPIN_Move(FWD, 800) is used to move 800 steps FWD, and
   immediately after that, with no soft stop between them, a dSPIN_Run(FWD, 200)
   command is issued, the 'run' command will execute with a speed based on the
   value in the 'MAX_SPEED' register, the way the 'move' command did, and NOT
   with the speed passed to it by the function call.
   
Initial release.

FILENAMES
dSPIN_h - This file. Constant definitions for register names, pin
   redefinitions, and bit field names.
dSPIN_commands.c - Contains high-level command implementations- movement
   and configuration commands, for example.
dSPIN_support.c - Contains functions used to implement the high-level commands,
   as well as utility functions for converting real-world units (eg, steps/s) to
   values usable by the dsPIN controller. Also contains the specialized configuration
   function for the dsPIN chip and the onboard peripherals needed to use it.
dSPIN_main.c - Contains a sanity test routine.
 *****************************************************************/

// emulate silly Arduino convention
typedef unsigned char byte;

// include the SPI library:
#include <wiringPi.h>
#include <wiringPiSPI.h>

// Pin settings are arbitrary and can be changed to any available
// digitalWrite capable pin.
#define dSPIN_RESET      24   // Wire this to the STBY line
#define dSPIN_BUSYN      25   // Wire this to the BSYN line
#define dSPIN_CS				 23		// Wire this to the CSN line
#define dSPIN_MOSI			 27		// Wire this to the CSN line
#define dSPIN_MISO			 22		// Wire this to the CSN line
#define dSPIN_CLK 			 17		// Wire this to the CSN line

#define dSPIN_SPI_CHANNEL 0

// constant definitions for overcurrent thresholds. Write these values to 
//  register dSPIN_OCD_TH to set the level at which an overcurrent even occurs.
#define dSPIN_OCD_TH_375mA  0x00
#define dSPIN_OCD_TH_750mA  0x01
#define dSPIN_OCD_TH_1125mA 0x02
#define dSPIN_OCD_TH_1500mA 0x03
#define dSPIN_OCD_TH_1875mA 0x04
#define dSPIN_OCD_TH_2250mA 0x05
#define dSPIN_OCD_TH_2625mA 0x06
#define dSPIN_OCD_TH_3000mA 0x07
#define dSPIN_OCD_TH_3375mA 0x08
#define dSPIN_OCD_TH_3750mA 0x09
#define dSPIN_OCD_TH_4125mA 0x0A
#define dSPIN_OCD_TH_4500mA 0x0B
#define dSPIN_OCD_TH_4875mA 0x0C
#define dSPIN_OCD_TH_5250mA 0x0D
#define dSPIN_OCD_TH_5625mA 0x0E
#define dSPIN_OCD_TH_6000mA 0x0F

// STEP_MODE option values.
// First comes the "microsteps per step" options...
#define dSPIN_STEP_MODE_STEP_SEL 0x07  // Mask for these bits only.
#define dSPIN_STEP_SEL_1     0x00
#define dSPIN_STEP_SEL_1_2   0x01
#define dSPIN_STEP_SEL_1_4   0x02
#define dSPIN_STEP_SEL_1_8   0x03
#define dSPIN_STEP_SEL_1_16  0x04
#define dSPIN_STEP_SEL_1_32  0x05
#define dSPIN_STEP_SEL_1_64  0x06
#define dSPIN_STEP_SEL_1_128 0x07

// ...next, define the SYNC_EN bit. When set, the BUSYN pin will instead
//  output a clock related to the full-step frequency as defined by the
//  SYNC_SEL bits below.
#define dSPIN_STEP_MODE_SYNC_EN	 0x80  // Mask for this bit
#define dSPIN_SYNC_EN 0x80

// ...last, define the SYNC_SEL modes. The clock output is defined by
//  the full-step frequency and the value in these bits- see the datasheet
//  for a matrix describing that relationship (page 46).
#define dSPIN_STEP_MODE_SYNC_SEL 0x70
#define dSPIN_SYNC_SEL_1_2 0x00
#define dSPIN_SYNC_SEL_1   0x10
#define dSPIN_SYNC_SEL_2   0x20
#define dSPIN_SYNC_SEL_4   0x30
#define dSPIN_SYNC_SEL_8   0x40
#define dSPIN_SYNC_SEL_16  0x50
#define dSPIN_SYNC_SEL_32  0x60
#define dSPIN_SYNC_SEL_64  0x70

// Bit names for the ALARM_EN register.
//  Each of these bits defines one potential alarm condition.
//  When one of these conditions occurs and the respective bit in ALARM_EN is set,
//  the FLAG pin will go low. The register must be queried to determine which event
//  caused the alarm.
#define dSPIN_ALARM_EN_OVERCURRENT       0x01
#define dSPIN_ALARM_EN_THERMAL_SHUTDOWN	 0x02
#define dSPIN_ALARM_EN_THERMAL_WARNING   0x04
#define dSPIN_ALARM_EN_UNDER_VOLTAGE     0x08
#define dSPIN_ALARM_EN_STALL_DET_A       0x10
#define dSPIN_ALARM_EN_STALL_DET_B       0x20
#define dSPIN_ALARM_EN_SW_TURN_ON        0x40
#define dSPIN_ALARM_EN_WRONG_NPERF_CMD   0x80

// CONFIG register renames.

// Oscillator options.
// The dSPIN needs to know what the clock frequency is because it uses that for some
//  calculations during operation.
#define dSPIN_CONFIG_OSC_SEL                 0x000F // Mask for this bit field.
#define dSPIN_CONFIG_INT_16MHZ               0x0000 // Internal 16MHz, no output
#define dSPIN_CONFIG_INT_16MHZ_OSCOUT_2MHZ   0x0008 // Default; internal 16MHz, 2MHz output
#define dSPIN_CONFIG_INT_16MHZ_OSCOUT_4MHZ   0x0009 // Internal 16MHz, 4MHz output
#define dSPIN_CONFIG_INT_16MHZ_OSCOUT_8MHZ   0x000A // Internal 16MHz, 8MHz output
#define dSPIN_CONFIG_INT_16MHZ_OSCOUT_16MHZ  0x000B // Internal 16MHz, 16MHz output
#define dSPIN_CONFIG_EXT_8MHZ_XTAL_DRIVE     0x0004 // External 8MHz crystal
#define dSPIN_CONFIG_EXT_16MHZ_XTAL_DRIVE    0x0005 // External 16MHz crystal
#define dSPIN_CONFIG_EXT_24MHZ_XTAL_DRIVE    0x0006 // External 24MHz crystal
#define dSPIN_CONFIG_EXT_32MHZ_XTAL_DRIVE    0x0007 // External 32MHz crystal
#define dSPIN_CONFIG_EXT_8MHZ_OSCOUT_INVERT  0x000C // External 8MHz crystal, output inverted
#define dSPIN_CONFIG_EXT_16MHZ_OSCOUT_INVERT 0x000D // External 16MHz crystal, output inverted
#define dSPIN_CONFIG_EXT_24MHZ_OSCOUT_INVERT 0x000E // External 24MHz crystal, output inverted
#define dSPIN_CONFIG_EXT_32MHZ_OSCOUT_INVERT 0x000F // External 32MHz crystal, output inverted

// Configure the functionality of the external switch input
#define dSPIN_CONFIG_SW_MODE                 0x0010 // Mask for this bit.
#define dSPIN_CONFIG_SW_HARD_STOP            0x0000 // Default; hard stop motor on switch.
#define dSPIN_CONFIG_SW_USER                 0x0010 // Tie to the GoUntil and ReleaseSW
                                                    //  commands to provide jog function.
                                                    //  See page 25 of datasheet.

// Configure the motor voltage compensation mode (see page 34 of datasheet)
#define dSPIN_CONFIG_EN_VSCOMP               0x0020  // Mask for this bit.
#define dSPIN_CONFIG_VS_COMP_DISABLE         0x0000  // Disable motor voltage compensation.
#define dSPIN_CONFIG_VS_COMP_ENABLE          0x0020  // Enable motor voltage compensation.

// Configure overcurrent detection event handling
#define dSPIN_CONFIG_OC_SD                   0x0080  // Mask for this bit.
#define dSPIN_CONFIG_OC_SD_DISABLE           0x0000  // Bridges do NOT shutdown on OC detect
#define dSPIN_CONFIG_OC_SD_ENABLE            0x0080  // Bridges shutdown on OC detect

// Configure the slew rate of the power bridge output
#define dSPIN_CONFIG_POW_SR                  0x0300  // Mask for this bit field.
#define dSPIN_CONFIG_SR_180V_us              0x0000  // 180V/us
#define dSPIN_CONFIG_SR_290V_us              0x0200  // 290V/us
#define dSPIN_CONFIG_SR_530V_us              0x0300  // 530V/us

// Integer divisors for PWM sinewave generation
//  See page 32 of the datasheet for more information on this.
#define dSPIN_CONFIG_F_PWM_DEC               0x1C00      // mask for this bit field
#define dSPIN_CONFIG_PWM_MUL_0_625           (0x00)<<10
#define dSPIN_CONFIG_PWM_MUL_0_75            (0x01)<<10
#define dSPIN_CONFIG_PWM_MUL_0_875           (0x02)<<10
#define dSPIN_CONFIG_PWM_MUL_1               (0x03)<<10
#define dSPIN_CONFIG_PWM_MUL_1_25            (0x04)<<10
#define dSPIN_CONFIG_PWM_MUL_1_5             (0x05)<<10
#define dSPIN_CONFIG_PWM_MUL_1_75            (0x06)<<10
#define dSPIN_CONFIG_PWM_MUL_2               (0x07)<<10

// Multiplier for the PWM sinewave frequency
#define dSPIN_CONFIG_F_PWM_INT               0xE000     // mask for this bit field.
#define dSPIN_CONFIG_PWM_DIV_1               (0x00)<<13
#define dSPIN_CONFIG_PWM_DIV_2               (0x01)<<13
#define dSPIN_CONFIG_PWM_DIV_3               (0x02)<<13
#define dSPIN_CONFIG_PWM_DIV_4               (0x03)<<13
#define dSPIN_CONFIG_PWM_DIV_5               (0x04)<<13
#define dSPIN_CONFIG_PWM_DIV_6               (0x05)<<13
#define dSPIN_CONFIG_PWM_DIV_7               (0x06)<<13

// Status register bit renames- read-only bits conferring information about the
//  device to the user.
#define dSPIN_STATUS_HIZ                     0x0001 // high when bridges are in HiZ mode
#define dSPIN_STATUS_BUSY                    0x0002 // mirrors BUSY pin
#define dSPIN_STATUS_SW_F                    0x0004 // low when switch open, high when closed
#define dSPIN_STATUS_SW_EVN                  0x0008 // active high, set on switch falling edge,
                                                    //  cleared by reading STATUS
#define dSPIN_STATUS_DIR                     0x0010 // Indicates current motor direction.
                                                    //  High is FWD, Low is REV.
#define dSPIN_STATUS_NOTPERF_CMD             0x0080 // Last command not performed.
#define dSPIN_STATUS_WRONG_CMD               0x0100 // Last command not valid.
#define dSPIN_STATUS_UVLO                    0x0200 // Undervoltage lockout is active
#define dSPIN_STATUS_TH_WRN                  0x0400 // Thermal warning
#define dSPIN_STATUS_TH_SD                   0x0800 // Thermal shutdown
#define dSPIN_STATUS_OCD                     0x1000 // Overcurrent detected
#define dSPIN_STATUS_STEP_LOSS_A             0x2000 // Stall detected on A bridge
#define dSPIN_STATUS_STEP_LOSS_B             0x4000 // Stall detected on B bridge
#define dSPIN_STATUS_SCK_MOD                 0x8000 // Step clock mode is active

// Status register motor status field
#define dSPIN_STATUS_MOT_STATUS                0x0060      // field mask
#define dSPIN_STATUS_MOT_STATUS_STOPPED       (0x0000)<<13 // Motor stopped
#define dSPIN_STATUS_MOT_STATUS_ACCELERATION  (0x0001)<<13 // Motor accelerating
#define dSPIN_STATUS_MOT_STATUS_DECELERATION  (0x0002)<<13 // Motor decelerating
#define dSPIN_STATUS_MOT_STATUS_CONST_SPD     (0x0003)<<13 // Motor at constant speed

// Register address redefines.
//  See the dSPIN_Param_Handler() function for more info about these.
#define dSPIN_ABS_POS              0x01
#define dSPIN_EL_POS               0x02
#define dSPIN_MARK                 0x03
#define dSPIN_SPEED                0x04
#define dSPIN_ACC                  0x05
#define dSPIN_DEC                  0x06
#define dSPIN_MAX_SPEED            0x07
#define dSPIN_MIN_SPEED            0x08
#define dSPIN_FS_SPD               0x15
#define dSPIN_KVAL_HOLD            0x09
#define dSPIN_KVAL_RUN             0x0A
#define dSPIN_KVAL_ACC             0x0B
#define dSPIN_KVAL_DEC             0x0C
#define dSPIN_INT_SPD              0x0D
#define dSPIN_ST_SLP               0x0E
#define dSPIN_FN_SLP_ACC           0x0F
#define dSPIN_FN_SLP_DEC           0x10
#define dSPIN_K_THERM              0x11
#define dSPIN_ADC_OUT              0x12
#define dSPIN_OCD_TH               0x13
#define dSPIN_STALL_TH             0x14
#define dSPIN_STEP_MODE            0x16
#define dSPIN_ALARM_EN             0x17
#define dSPIN_CONFIG               0x18
#define dSPIN_STATUS               0x19

//dSPIN commands
#define dSPIN_NOP                  0x00
#define dSPIN_SET_PARAM            0x00
#define dSPIN_GET_PARAM            0x20
#define dSPIN_RUN                  0x50
#define dSPIN_STEP_CLOCK           0x58
#define dSPIN_MOVE                 0x40
#define dSPIN_GOTO                 0x60
#define dSPIN_GOTO_DIR             0x68
#define dSPIN_GO_UNTIL             0x82
#define dSPIN_RELEASE_SW           0x92
#define dSPIN_GO_HOME              0x70
#define dSPIN_GO_MARK              0x78
#define dSPIN_RESET_POS            0xD8
#define dSPIN_RESET_DEVICE         0xC0
#define dSPIN_SOFT_STOP            0xB0
#define dSPIN_HARD_STOP            0xB8
#define dSPIN_SOFT_HIZ             0xA0
#define dSPIN_HARD_HIZ             0xA8
#define dSPIN_GET_STATUS           0xD0

/* dSPIN direction options */
#define FWD  0x01
#define REV  0x00

/* dSPIN action options */
#define ACTION_RESET  0x00
#define ACTION_COPY   0x01

/* basic error codes */
#define dSPIN_STATUS_GOOD
#define dSPIN_STATUS_FATAL 1

/*SPI clock settings */
#define dSPIN_SPI_CLOCK_SPD_1MHZ 1000000

/************ dSPIN_support.c ***********************/

/* Call this first to set up raspi SPI interface and prepare dSPIN to
 * receive commands.
 */
int dSPIN_init();
	
/* This simple function shifts a byte out over SPI and receives a byte over
 *  SPI. Unusually for SPI devices, the dSPIN requires a toggling of the
 *  CS (slaveSelect) pin after each byte sent. That makes this function
 *  a bit more reasonable, because we can include more functionality in it.
 */
byte dSPIN_Xfer(byte data);

// The value in the ACC register is [(steps/s/s)*(tick^2)]/(2^-40) where tick is 
//  250ns (datasheet value)- 0x08A on boot.
// Multiply desired steps/s/s by .137438 to get an appropriate value for this register.
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long AccCalc(float stepsPerSecPerSec);

// The calculation for DEC is the same as for ACC. Value is 0x08A on boot.
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long DecCalc(float stepsPerSecPerSec);

// The value in the MAX_SPD register is [(steps/s)*(tick)]/(2^-18) where tick is 
//  250ns (datasheet value)- 0x041 on boot.
// Multiply desired steps/s by .065536 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure it remains at or below 0x3FF
unsigned long MaxSpdCalc(float stepsPerSec);

// The value in the MIN_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
//  250ns (datasheet value)- 0x000 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long MinSpdCalc(float stepsPerSec);

// The value in the FS_SPD register is ([(steps/s)*(tick)]/(2^-18))-0.5 where tick is 
//  250ns (datasheet value)- 0x027 on boot.
// Multiply desired steps/s by .065536 and subtract .5 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure the value is at or below 0x3FF.
unsigned long FSCalc(float stepsPerSec);

// The value in the INT_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
//  250ns (datasheet value)- 0x408 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 14-bit value, so we need to make sure the value is at or below 0x3FFF.
unsigned long IntSpdCalc(float stepsPerSec);

// When issuing RUN command, the 20-bit speed is [(steps/s)*(tick)]/(2^-28) where tick is 
//  250ns (datasheet value).
// Multiply desired steps/s by 67.106 to get an appropriate value for this register
// This is a 20-bit value, so we need to make sure the value is at or below 0xFFFFF.
unsigned long SpdCalc(float stepsPerSec);

// Generalization of the subsections of the register read/write functionality.
//  We want the end user to just write the value without worrying about length,
//  so we pass a bit length parameter from the calling function.
unsigned long dSPIN_Param(unsigned long value, byte bit_len);

/************ dSPIN_commands.c ***********************/

// Realize the "set parameter" function, to write to the various registers in
//  the dSPIN chip.
void dSPIN_SetParam(byte param, unsigned long value);

// Realize the "get parameter" function, to read from the various registers in
//  the dSPIN chip.
unsigned long dSPIN_GetParam(byte param);

// Enable or disable the low-speed optimization option. If enabling,
//  the other 12 bits of the register will be automatically zero.
//  When disabling, the value will have to be explicitly written by
//  the user with a SetParam() call. See the datasheet for further
//  information about low-speed optimization.
void SetLSPDOpt(bool enable);

// RUN sets the motor spinning in a direction (defined by the constants
//  FWD and REV). Maximum speed and minimum speed are defined
//  by the MAX_SPEED and MIN_SPEED registers; exceeding the FS_SPD value
//  will switch the device into full-step mode.
// The SpdCalc() function is provided to convert steps/s values into
//  appropriate integer values for this function.
void dSPIN_Run(byte dir, unsigned long spd);

// STEP_CLOCK puts the device in external step clocking mode. When active,
//  pin 25, STCK, becomes the step clock for the device, and steps it in
//  the direction (set by the FWD and REV constants) imposed by the call
//  of this function. Motion commands (RUN, MOVE, etc) will cause the device
//  to exit step clocking mode.
void dSPIN_Step_Clock(byte dir);

// MOVE will send the motor n_step steps (size based on step mode) in the
//  direction imposed by dir (FWD or REV constants may be used). The motor
//  will accelerate according the acceleration and deceleration curves, and
//  will run at MAX_SPEED. Stepping mode will adhere to FS_SPD value, as well.
void dSPIN_Move(byte dir, unsigned long n_step);

// GOTO operates much like MOVE, except it produces absolute motion instead
//  of relative motion. The motor will be moved to the indicated position
//  in the shortest possible fashion.
void dSPIN_GoTo(unsigned long pos);

// GoUntil will set the motor running with direction dir (REV or
//  FWD) until a falling edge is detected on the SW pin. Depending
//  on bit SW_MODE in CONFIG, either a hard stop or a soft stop is
//  performed at the falling edge, and depending on the value of
//  act (either RESET or COPY) the value in the ABS_POS register is
//  either RESET to 0 or COPY-ed into the MARK register.
void dSPIN_GoUntil(byte act, byte dir, unsigned long spd);

// Similar in nature to GoUntil, ReleaseSW produces motion at the
//  higher of two speeds: the value in MIN_SPEED or 5 steps/s.
//  The motor continues to run at this speed until a rising edge
//  is detected on the switch input, then a hard stop is performed
//  and the ABS_POS register is either COPY-ed into MARK or RESET to
//  0, depending on whether RESET or COPY was passed to the function
//  for act.
void dSPIN_ReleaseSW(byte act, byte dir);

// GoHome is equivalent to GoTo(0), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void dSPIN_GoHome();

// GoMark is equivalent to GoTo(MARK), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void dSPIN_GoMark();

// Sets the ABS_POS register to 0, effectively declaring the current
//  position to be "HOME".
void dSPIN_ResetPos();

// Reset device to power up conditions. Equivalent to toggling the STBY
//  pin or cycling power.
void dSPIN_ResetDev();

// Bring the motor to a halt using the deceleration curve.
void dSPIN_SoftStop();

// Stop the motor with infinite deceleration.
void dSPIN_HardStop();

// Decelerate the motor and put the bridges in Hi-Z state.
void dSPIN_SoftHiZ();

// Put the bridges in Hi-Z state immediately with no deceleration.
void dSPIN_HardHiZ();

// Fetch and return the 16-bit value in the STATUS register. Resets
//  any warning flags and exits any error states. Using GetParam()
//  to read STATUS does not clear these values.
int dSPIN_GetStatus();
