/*
 * Card_Routines.h
 *
 *  Created on: Sep 8, 2014
 *      Author: Tanmay
 */

#ifndef CARD_ROUTINES_H_
#define CARD_ROUTINES_H_


#ifdef __cplusplus
extern "C"
{
#endif

struct FileRoutines{
	int drv;
	F_FILE* fp;
};

int *initOnBoardSD(int drv);

#ifdef __cplusplus
}
#endif


#endif /* CARD_ROUTINES_H_ */
