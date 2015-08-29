/*
 * Accessory_Functions.cpp
 *
 *  Created on: Oct 28, 2014
 *      Author: Tanmay
 */
///included
#include "predef.h"
#include <stdio.h>
#include <stdlib.h>
#include <basictypes.h>
#include <ucos.h>
#include <ctype.h>
#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
#include <taskmon.h>
#include <smarttrap.h>
#include <effs_fat/fat.h>
#include <effs_fat/multi_drive_mmc_mcf.h>
#include <effs_fat/effs_utils.h>
#include <sim.h>
#include <pins.h>
#include <ucosmcfc.h>
#include <pinconstant.h>
#include <HiResTimer.h>
#include <utils.h>
#include <constants.h>
#include <cfinter.h>
#include <math.h>
#include <serial.h>
#include <dspi.h> //needed for IMU communication
#include "PINS_Definations.h"
#include "FileSystemUtils.h"
#include "Card_Routines.h"
#include <pitr_sem.h>//for PIT SEM
#include "Accessory_Functions.h" //Housekeeping functions





void Usual_Routine() {
	InitializeStack();
	OSChangePrio(MAIN_PRIO);
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();
	initPINS();

	J2[32] = 1; //Resets the OSD

	OSTimeDly(2);

	DSPIInit(3, 2000000, 16, 0x00, 1, 1, 1, 0, 0, 0); //Initializing the Hardware to talk to IMU
	DSPIInit(1, 2000000, 16, 0x00, 0x01, 0, 0, 0, 0, 0); //Initializing the Hardware to talk to OSD

	/***********************************Initializing Initial Artificial Horizon*****************************************************************/

	uint16_t x = 2834; //Center Position of the Center Circle
	Display_Center(x); //Displaying the Center, argument center address
	Display_Data(); //Initializing Pitch and Roll value Display
}

void OpenFileRoutine(struct FileRoutines* OpenOnboardSD) {

	char File_name[20] = { 0 };
	(*OpenOnboardSD).drv = OpenOnBoardFlash();
	int *card_status = initOnBoardSD((*OpenOnboardSD).drv);
	uint8_t n = card_status[1] + 1;
	siprintf(File_name, "LOG%d.txt", n);

	(*OpenOnboardSD).fp = f_open(File_name, "w+");

}



int SerialRoutine(int port_num) {
	SerialClose(port_num);
	return (OpenSerial(port_num, 115200, 1, 8, eParityNone));
}


HiResTimer* InitTimer(int clock_number) {
	HiResTimer* timer;
	timer = HiResTimer::getHiResTimer(clock_number); //for keeping time
	timer->init();
	timer->start();
	return (timer);
}

void configIMU() {

	OSTimeDly(TICKS_PER_SECOND * 2);

	BYTE IMU_config[16] = { 0x80, 0x03, 0x8C, 0x13, 0x8D, 0x00, 0x8E, 0x0A,//change 3rd index (array starts at 0 index) to change the SPS
			0x8F, 0x07, 0x82, 0x01, 0x83, 0x00, 0x80, 0x00 }; //Configs IMU decimation rate value to 19 so Gyro output is 123 SPS (2460/(19+1))
	//configure IMU
	DSPIStart(3, IMU_config, NULL, sizeof(IMU_config), NULL);
	while (!DSPIdone(3)) {
	};
}



