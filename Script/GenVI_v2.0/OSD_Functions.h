/*
 * OSD_Functions.h
 *
 *  Created on: Aug 15, 2014
 *      Author: Tanmay
 */

#include <basictypes.h>

#ifndef OSD_FUNCTIONS_H_
#define OSD_FUNCTIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

void initOSD(void);

/*************Tanmay Coordinates*********************
 * A=Tanmay Address
 * P=A / 30 (just the quotient)
 * C=A % 30
 * R= P / 17 (just the quotient)
 * b=P % 17 (this the the row of the pixel you want to display from datasheet each display pixel has 18 rows numbered from 0-17
 * so this b represents which row in a display pixel you want to show) this display pixels were custom created from 0x50 character memory address
 *
 * Then Actual Display Address Ad=30*R+29+(C+1)
 * Since in AH(Artificial Horizon) the column number is constant therefore C here is 14 but the formula above is for any general address
 */

static inline int fast_absInt(int x) {
	return (x >= 0) ? x : -x;
}

static inline void  DisplayCharacter(uint8_t L_Add, uint8_t Character ){//function to display a character, argument Lower Byte of the OSD Display address and the character memory address

	BYTE OSD_DOUT[2]={0};

	OSD_DOUT[0]=0x06;OSD_DOUT[1] =L_Add;//Lower byte of the diplay address
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish


	OSD_DOUT[0]=0x07;OSD_DOUT[1] =Character;//Character to Display
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//
	while(!DSPIdone(1)){};//wait for DSPI to finish*/


}


void Enable_OSD(void); //Enables OSD Display
static inline void OSD_Position_H(uint8_t H_Add){

	BYTE OSD_DOUT[2]={0};

	OSD_DOUT[0]=0x05;OSD_DOUT[1] =H_Add; //high bit of OSD display address
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAH
	while(!DSPIdone(1)){};//wait for DSPI to finish
} //Decides the high bit of OSD display address

static inline void Display_Center(uint16_t x){

	uint8_t pixel_row=0,column_no=0,bar_no=0,actual_row=0;;


	//computing actual address from Tanmay Coordinates
	pixel_row=x / 30;
	column_no=x % 30;
	actual_row=pixel_row / 17;
	bar_no= pixel_row % 17;

	uint16_t Disp_Add=30*actual_row+29+(column_no+1);//Actual address
	uint8_t Disp_Char=0xC2;

	if(Disp_Add >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(Disp_Add,0xC0);//Display the center O

	if((Disp_Add-1) >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(Disp_Add-1,Disp_Char);//Display the left wing

	if((Disp_Add+1) >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(Disp_Add+1,Disp_Char);//Display the right wing
} //Displays the center circle, center address in Tanmay Coordinates

static inline void Display_Center_Line(uint16_t x,int8_t roll){//Display Center Line


	float froll=roll*0.1;
	int16_t pixel_row=0,column_no=0,bar_no=0,actual_row=0;
	uint16_t Disp_Add=0;
	uint8_t Disp_Char=0;


	for (int i=-9;i<10;i++){//for loop to Display 19 lines to draw center Line

		pixel_row=(x-i*30*froll) / 30;//Computing actuall address from Tanmay Address
		if(pixel_row>-1){//making sure we don't have negative display address
			column_no=14;
			actual_row=pixel_row / 17;
			bar_no= pixel_row % 17;

			Disp_Add=30*actual_row+29+(column_no+1);
			Disp_Char=128+bar_no;


			if(Disp_Add< 480){//making sure that we don't have any address above 480
				if ((Disp_Add+i)>255)
					OSD_Position_H(0x01);
				else
					OSD_Position_H(0x00);
				DisplayCharacter(Disp_Add+i,Disp_Char);//////////////////Center Line
			}


		}
	}

} //Displays the Center line or Horizon,Address in Tanmay Coordinates and roll value

static inline void Remove_Center_Line(uint16_t x,int8_t roll){//Remove Center Line //See Display Center Line for more info.


	float froll=roll*0.1;
	int16_t pixel_row=0,column_no=0,bar_no=0,actual_row=0;
	uint16_t Disp_Add=0;
	uint8_t Disp_Char=0;


	for (int i=-9;i<10;i++){

		pixel_row=(x-i*30*froll) / 30;
		if(pixel_row>-1){
			column_no=14;
			actual_row=pixel_row / 17;
			bar_no= pixel_row % 17;

			Disp_Add=30*actual_row+29+(column_no+1);
			Disp_Char=128+bar_no;


			if(Disp_Add< 480){
				if ((Disp_Add+i)>255)
					OSD_Position_H(0x01);
				else
					OSD_Position_H(0x00);
				DisplayCharacter(Disp_Add+i,0x00);//Blank Character

			}


		}
	}
} //Removes the center line, address in Tanmay Coordinates and roll angle

static inline void Replace_Center_Line(int8_t i, int8_t j){//Calls Remove and Display Center Line function
	static int y_pitch=2834;
	static int y_roll=0;

	uint16_t x=2834+i*30;

	Remove_Center_Line(y_pitch,y_roll);//removes the old line
	Display_Center_Line(x,j);//displays the new line
	Display_Center(2834);//just to be sure the center is always pressent

	y_pitch=x;
	y_roll=j;

} //Removes and redraws the center line, picth and roll input


void Replace_Character(uint8_t Char_Add); //Replaces a character
void Display_Data(void); //Displays the Text Data

static inline void Display_Roll(int8_t roll) { //Displaying the roll data

	int f_char = 0;
	int s_char = 0;

	if (roll < 0) //Deciding which sign to display
		DisplayCharacter(57, 0x49);
	if (roll >= 0)
		DisplayCharacter(57, 0x50);

	f_char = fast_absInt(roll / 10); //Showing just the integer value
	//f_char---->First Character
	if (f_char != 0) //check for 0
		DisplayCharacter(58, f_char);
	else
		DisplayCharacter(58, 0x0A);

	s_char = fast_absInt(roll % 10);
	//s_char----->Second Character
	if (s_char != 0) //Check for 0
		DisplayCharacter(59, s_char);
	else
		DisplayCharacter(59, 0x0A);

}

static inline void Display_Pitch(int8_t pitch) { //Same as Roll but with Pitch angle

	int f_char = 0;
	int s_char = 0;

	if (pitch < 0)
		DisplayCharacter(27, 0x49);
	if (pitch >= 0)
		DisplayCharacter(27, 0x50);

	f_char = fast_absInt(pitch / 10);
	if (f_char != 0)
		DisplayCharacter(28, f_char);
	else
		DisplayCharacter(28, 0x0A);

	s_char = fast_absInt(pitch % 10);
	if (s_char != 0)
		DisplayCharacter(29, s_char);
	else
		DisplayCharacter(29, 0x0A);
}

#ifdef __cplusplus
}
#endif

#endif /* OSD_FUNCTIONS_H_ */
