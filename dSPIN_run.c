//dSPIN_test_jig.c - Basic check of the library and demonstration of its
//										most important functions
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dSPIN.h"

#define SOFT_STOP 0
#define HARD_STOP 1

int runSpeed = 200;
int dir = FWD;

void stop(bool);
void run(int, int);
void move( int dist, int dir, int speed);

int main(int argc, char* argv[]){  

  // dSPIN_init() is implemented in the dSPIN_support.c file. It includes
  //  all the necessary port setup and SPI setup to allow the Raspberry Pi to
  //  control the dSPIN chip and relies entirely upon the pin redefinitions
  //  in dSPIN.h
  dSPIN_init();

	if(argc>1){
		if(argv[1][0]=='0'){
			printf("stopping motor\n");
			if (!strcmp(argv[2], "soft" )){
				stop(SOFT_STOP);
				printf("softly\n");
			}else
				stop(HARD_STOP);
		// if 1st arg is for, use next arg as dist, not speed
		} else if (!strcmp(argv[1],"for")){
			int dist = atoi(argv[1]);
			if(dist<0){
				dir = REV;
				dist = 0-dist;
			}
			printf("moving %i steps.\n", dist);
			move( atoi(argv[2]), dir, 20);

		}else{ 		
			runSpeed = atoi(argv[1]);
			if(runSpeed<0){
				dir = REV;
				runSpeed = 0-runSpeed;
			}
			printf("starting motor with speed %i.\n", runSpeed);
			run(runSpeed, dir);
		}
	}
}


void stop(bool hard) {
	if(hard) dSPIN_HardHiZ();
	else dSPIN_SoftHiZ();
}

void move( int dist, int dir, int speed){
  
  // First things first: let's check communications. The CONFIG register should
  //  power up to 0x2E88, so we can use that to check the communications.
  //  On the test jig, this causes an LED to light up.
  if (dSPIN_GetParam(dSPIN_CONFIG) == 0x2E88) 
		printf("Configuration	successful!");
  
  // The following function calls are for this demo application- you will
  //  need to adjust them for your particular application, and you may need
  //  to configure additional registers.
  
  // First, let's set the step mode register:
  //   - dSPIN_SYNC_EN controls whether the BUSY/SYNC pin reflects the step
  //     frequency or the BUSY status of the chip. We want it to be the BUSY
  //     status.
  //   - dSPIN_STEP_SEL_x is the microstepping rate- we'll go full step.
  //   - dSPIN_SYNC_SEL_x is the ratio of (micro)steps to toggles on the
  //     BUSY/SYNC pin (when that pin is used for SYNC). Make it 1:1, despite
  //     not using that pin.
  dSPIN_SetParam(dSPIN_STEP_MODE, !dSPIN_SYNC_EN | dSPIN_STEP_SEL_1 | dSPIN_SYNC_SEL_1);
  // Configure the MAX_SPEED register- this is the maximum number of (micro)steps per
  //  second allowed. You'll want to mess around with your desired application to see
  //  how far you can push it before the motor starts to slip. The ACTUAL parameter
  //  passed to this function is in steps/tick; MaxSpdCalc() will convert a number of
  //  steps/s into an appropriate value for this function. Note that for any move or
  //  goto type function where no speed is specified, this value will be used.
  dSPIN_SetParam(dSPIN_MAX_SPEED, MaxSpdCalc(speed));
  // Configure the FS_SPD register- this is the speed at which the driver ceases
  //  microstepping and goes to full stepping. FSCalc() converts a value in steps/s
  //  to a value suitable for this register; to disable full-step switching, you
  //  can pass 0x3FF to this register.
  //dSPIN_SetParam(dSPIN_FS_SPD, FSCalc(150));
  dSPIN_SetParam(dSPIN_FS_SPD, 0x3FF);
  // Configure the acceleration rate, in steps/tick/tick. There is also a DEC register;
  //  both of them have a function (AccCalc() and DecCalc() respectively) that convert
  //  from steps/s/s into the appropriate value for the register. Writing ACC to 0xfff
  //  sets the acceleration and deceleration to 'infinite' (or as near as the driver can
  //  manage). If ACC is set to 0xfff, DEC is ignored. To get infinite deceleration
  //  without infinite acceleration, only hard stop will work.
  dSPIN_SetParam(dSPIN_ACC, 0x040);
  // Configure the overcurrent detection threshold. The constants for this are defined
  //  in the dSPIN.h file.
	// 3000mA is somewhere a bit above the rated capacity w/o heatsinking.
  dSPIN_SetParam(dSPIN_OCD_TH, dSPIN_OCD_TH_1875mA);
  // Set up the CONFIG register as follows:
  //  PWM frequency divisor = 1
  //  PWM frequency multiplier = 2 (62.5kHz PWM frequency)
  //  Slew rate is 290V/us
  //  Do NOT shut down bridges on overcurrent
  //  Disable motor voltage compensation
  //  Hard stop on switch low
  //  16MHz internal oscillator, nothing on output
  dSPIN_SetParam(dSPIN_CONFIG, 
                   dSPIN_CONFIG_PWM_DIV_1 | dSPIN_CONFIG_PWM_MUL_2 | dSPIN_CONFIG_SR_290V_us
                 | dSPIN_CONFIG_OC_SD_ENABLE | dSPIN_CONFIG_VS_COMP_DISABLE 
                 | dSPIN_CONFIG_SW_USER | dSPIN_CONFIG_INT_16MHZ);
  // Configure the RUN KVAL. This defines the duty cycle of the PWM of the bridges
  //  during running. 0xFF means that they are essentially NOT PWMed during run; this
  //  MAY result in more power being dissipated than you actually need for the task.
  //  Setting this value too low may result in failure to turn.
  //  There are ACC, DEC, and HOLD KVAL registers as well; you may need to play with
  //  those values to get acceptable performance for a given application.
  dSPIN_SetParam(dSPIN_KVAL_RUN, 0xaF);
  // Calling GetStatus() clears the UVLO bit in the status register, which is set by
  //  default on power-up. The driver may not run without that bit cleared by this
  //  read operation.
  printf("Status code is: %x\n"
				 "It shouldn't be 0, but this is normal at the moment.\n", dSPIN_GetStatus());
  while (digitalRead(dSPIN_BUSYN) == LOW);
  delay(500);
  dSPIN_Move(dir, dist ); 


}

void run( int speed, int dir){
  
  // First things first: let's check communications. The CONFIG register should
  //  power up to 0x2E88, so we can use that to check the communications.
  //  On the test jig, this causes an LED to light up.
  if (dSPIN_GetParam(dSPIN_CONFIG) == 0x2E88) 
		printf("Configuration	successful!");
  
  // The following function calls are for this demo application- you will
  //  need to adjust them for your particular application, and you may need
  //  to configure additional registers.
  
  // First, let's set the step mode register:
  //   - dSPIN_SYNC_EN controls whether the BUSY/SYNC pin reflects the step
  //     frequency or the BUSY status of the chip. We want it to be the BUSY
  //     status.
  //   - dSPIN_STEP_SEL_x is the microstepping rate- we'll go full step.
  //   - dSPIN_SYNC_SEL_x is the ratio of (micro)steps to toggles on the
  //     BUSY/SYNC pin (when that pin is used for SYNC). Make it 1:1, despite
  //     not using that pin.
  dSPIN_SetParam(dSPIN_STEP_MODE, !dSPIN_SYNC_EN | dSPIN_STEP_SEL_1 | dSPIN_SYNC_SEL_1);
  // Configure the MAX_SPEED register- this is the maximum number of (micro)steps per
  //  second allowed. You'll want to mess around with your desired application to see
  //  how far you can push it before the motor starts to slip. The ACTUAL parameter
  //  passed to this function is in steps/tick; MaxSpdCalc() will convert a number of
  //  steps/s into an appropriate value for this function. Note that for any move or
  //  goto type function where no speed is specified, this value will be used.
  dSPIN_SetParam(dSPIN_MAX_SPEED, MaxSpdCalc(290));
  // Configure the FS_SPD register- this is the speed at which the driver ceases
  //  microstepping and goes to full stepping. FSCalc() converts a value in steps/s
  //  to a value suitable for this register; to disable full-step switching, you
  //  can pass 0x3FF to this register.
  dSPIN_SetParam(dSPIN_FS_SPD, FSCalc(150));
  // Configure the acceleration rate, in steps/tick/tick. There is also a DEC register;
  //  both of them have a function (AccCalc() and DecCalc() respectively) that convert
  //  from steps/s/s into the appropriate value for the register. Writing ACC to 0xfff
  //  sets the acceleration and deceleration to 'infinite' (or as near as the driver can
  //  manage). If ACC is set to 0xfff, DEC is ignored. To get infinite deceleration
  //  without infinite acceleration, only hard stop will work.
  dSPIN_SetParam(dSPIN_ACC, 0x040);
  // Configure the overcurrent detection threshold. The constants for this are defined
  //  in the dSPIN.h file.
	// 3000mA is somewhere a bit above the rated capacity w/o heatsinking.
  dSPIN_SetParam(dSPIN_OCD_TH, dSPIN_OCD_TH_1875mA);
  // Set up the CONFIG register as follows:
  //  PWM frequency divisor = 1
  //  PWM frequency multiplier = 2 (62.5kHz PWM frequency)
  //  Slew rate is 290V/us
  //  Do NOT shut down bridges on overcurrent
  //  Disable motor voltage compensation
  //  Hard stop on switch low
  //  16MHz internal oscillator, nothing on output
  dSPIN_SetParam(dSPIN_CONFIG, 
                   dSPIN_CONFIG_PWM_DIV_1 | dSPIN_CONFIG_PWM_MUL_2 | dSPIN_CONFIG_SR_290V_us
                 | dSPIN_CONFIG_OC_SD_ENABLE | dSPIN_CONFIG_VS_COMP_DISABLE 
                 | dSPIN_CONFIG_SW_USER | dSPIN_CONFIG_INT_16MHZ);
  // Configure the RUN KVAL. This defines the duty cycle of the PWM of the bridges
  //  during running. 0xFF means that they are essentially NOT PWMed during run; this
  //  MAY result in more power being dissipated than you actually need for the task.
  //  Setting this value too low may result in failure to turn.
  //  There are ACC, DEC, and HOLD KVAL registers as well; you may need to play with
  //  those values to get acceptable performance for a given application.
  dSPIN_SetParam(dSPIN_KVAL_RUN, 0xaF);
  // Calling GetStatus() clears the UVLO bit in the status register, which is set by
  //  default on power-up. The driver may not run without that bit cleared by this
  //  read operation.
  printf("Status code is: %x\n"
				 "It shouldn't be 0, but this is normal at the moment.\n", dSPIN_GetStatus());
  while (digitalRead(dSPIN_BUSYN) == LOW);
  delay(500);
  dSPIN_Run(dir, SpdCalc(speed)); 



}
