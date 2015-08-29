/*
 * OSD_Functions.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: Tanmay
 */


//included
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
#include "OSD_Functions.h"

void initOSD(){

	BYTE OSD_DIN[8]={0};
	BYTE OSD_DOUT[2]={0};

	OSD_DOUT[0]=0xEC;OSD_DOUT[1]=0x00;
	DSPIStart(1,OSD_DOUT,OSD_DIN,2,NULL);//Get Image Black Level
	while(!DSPIdone(1)){};//wait for DSPI to finish

	//OSTimeDly(5);

	OSD_DOUT[0]=0x6C;OSD_DOUT[1] =OSD_DIN[1] & 0xEF;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Set OSD Black level to Image Black level
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSTimeDly(5);

	OSD_DOUT[0]=0x04;OSD_DOUT[1] =0x00;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMM set to 0
	while(!DSPIdone(1)){};//wait for DSPI to finish

}



void Enable_OSD(){

	BYTE OSD_Enable[2]={0x00,0x08};
	DSPIStart(1,OSD_Enable,NULL,2,NULL);//Enable display of OSD image
	while(!DSPIdone(1)){};//wait for DSPI to finish
}












void Display_Data(void){//Setting up text display

	OSD_Position_H(0x00);
	DisplayCharacter(25,0x1A);
	DisplayCharacter(26,0x44);

	DisplayCharacter(55,0x1C);
	DisplayCharacter(56,0x44);
}




