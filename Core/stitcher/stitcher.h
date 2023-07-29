/*
 * stitcher.h
 *
 *  Created on: Aug 22, 2022
 *      Author: sanjay
 */

#ifndef STITCHER_STITCHER_H_
#define STITCHER_STITCHER_H_


struct stitchConfig_t
{
	unsigned short int stitchSpeed;
	unsigned short int stepX, stepY;
	unsigned short int timeX, timeY;
};


void stitcherInit();

int makeStitch(stitchConfig_t *cfg);


#endif /* STITCHER_STITCHER_H_ */
