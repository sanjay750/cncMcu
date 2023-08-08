/*
 * PLC_DEF.h
 *
 *  Created on: 15-May-2023
 *      Author: sanjay
 */

#ifndef INC_PLC_DEF_H_
#define INC_PLC_DEF_H_


#define ZERO_SENSOR X0
#define NEEDLE_TOP_SENSOR 		X1
#define NEEDLE_POS_SENSOR 		X2
#define NEEDLE_DOWN_SENSOR 		X3
#define START_BUTTON		 	X4
#define STOP_BUTTON	 			X5
#define QUILT_SIGNAL		 	X6
//#define N X7
//#define N X8
//#define N X9
//#define N X10
//#define N X11
//#define N X12
//#define N X13
//#define N X14
//#define N X15



#define VFD_RUN 				Y0
#define VFD_M1 					Y1
#define VFD_M2 					Y2
#define VFD_M3 					Y3
#define VFD_M4 					Y4
#define MEG_BREAK 				Y5

#define LOWER_THREAD_VALVE 		Y6
#define UPPER_THREAD_VALVE 		Y7
#define LOOPER_VALVE			Y8

#define LATCH_QUILT_OUT				Y9
#define UNLATCH_QUILT_OUT 			Y10
//#define N Y11
//#define N Y12
//#define N Y13
//#define N Y14
//#define N Y15

#define MOVE_RIGHT		 M0
#define MOVE_LEFT		 M1
#define MOVE_UP			 M2
#define MOVE_DOWN		 M3
#define NEEDLE_UP 		M4
#define NEEDLE_DOWN		 M5
#define NEEDLE_LOOP		M6
//#define N M7
//#define N M8
//#define N M9
//#define N M10
//#define N M11
//#define N M12
#define TEST_BREAK	 M13
//#define N M14
//#define N M15
//#define N M16
//#define N M17
//#define N M18
#define RUN_FDR		 	M19
#define FDR_RING 		M20
#define INS_QNF		 	M21
#define FDR_HART_BEAT  	M22
#define FDR_ALIVE		M23
#define RUN_QUILT		M24
#define CNC_IDEL_STATE  M25
#define CNC_STOP_STATE 	M26
#define CNC_ERROR_STATE M27
#define CNC_ZERO_STATE M28
//#define CNC_STITCH_STATE M29
//#define CNC_STITCH_STATE M30
//#define N M31

#define SIM_QUILT_SIG	M31

#define VFD_SPEED_REQUEST	D2
#define VFD_SPEED			D3

typedef enum
{
	X_NOPE = 0xFFFF,
	X0 = 0,
	X1,
	X2,
	X3,
	X4,
	X5,
	X6,
	X7,
	X8,
	X9,
	X10,
	X11,
	X12,
	X13,
	X14,
	X15,

}input_t;

typedef enum
{
	M_NOPE = 0,
	M0 = (1LL<<0), // move x+
	M1 = (1LL<<1), // move x-
	M2 = (1LL<<2), // move y+
	M3 = (1LL<<3), // move y-
	M4 = (1LL<<4), // needle top
	M5 = (1LL<<5), // needle down
	M6 = (1LL<<6), // needle loop
	M7 = (1LL<<7), // move to zero

	M8 = (1LL<<8), // 0:ZERO sensor NPN 1: ZERO sensor PNP
	M9 = (1LL<<9), // 0:TOP sensor NPN 1: TOP sensor PNP
	M10 = (1LL<<10), // 0:POS sensor NPN 1: POS sensor PNP
	M11 = (1LL<<11), // 0:DOWN sensor NPN 1: DOWN sensor PNP
	M12 = (1LL<<12), // 0:PROCT sensor NPN 1: PROCT sensor PNP


	M13 = (1LL<<13), // test break
	M14 = (1LL<<14), // test lower thread loose valve
	M15 = (1LL<<15), // test upper thread loose valve
	M16 = (1LL<<16), // test looper valve
	M17 = (1LL<<17), // test quilt latch
	M18 = (1LL<<18), // test quilt unlatch

	M19 = (1LL<<19), // run feeder command
	M20 = (1LL<<20), // feeder running status
	M21 = (1LL<<21), // ins queue not full
	M22 = (1LL<<22),
	M23 = (1LL<<23),
	M24 = (1LL<<24),
	M25 = (1LL<<25),
	M26 = (1LL<<26),
	M27 = (1LL<<27),
	M28 = (1LL<<28),
	M29 = (1LL<<29),
	M30 = (1LL<<30),
	M31 = (1LL<<31),
	M32 = (1LL<<32),
	M33 = (1LL<<33),
	M34 = (1LL<<34),
	M35 = (1LL<<35),
	M36 = (1LL<<36),
	M37 = (1LL<<37),
	M38 = (1LL<<38),
	M39 = (1LL<<39),
	M40 = (1LL<<40),
	M41 = (1LL<<41),
	M42 = (1LL<<42),
	M43 = (1LL<<43),
	M44 = (1LL<<44),
	M45 = (1LL<<45),
	M46 = (1LL<<46),
	M47 = (1LL<<47),
	M48 = (1LL<<48),
	M49 = (1LL<<49),
	M50 = (1LL<<50),
	M51 = (1LL<<51),
	M52 = (1LL<<52),
	M53 = (1LL<<53),
	M54 = (1LL<<54),
	M55 = (1LL<<55),
	M56 = (1LL<<56),
	M57 = (1LL<<57),
	M58 = (1LL<<58),
	M59 = (1LL<<59),
	M60 = (1LL<<60),
	M61 = (1LL<<61),
	M62 = (1LL<<62),
	M63 = (1LL<<63),


}auxRelay_t;




typedef enum
{
	Y_NOPE = 0xFFFF,
	Y0 = 0,
	Y1,
	Y2,
	Y3,
	Y4,
	Y5,
	Y6,
	Y7,
	Y8,
	Y9,
	Y10,
	Y11,
	Y12,
	Y13,
	Y14,
	Y15,



}output_t;

typedef enum
{
	I0,
	I1,
	I2,
	I3,
	I4, //xServo range
	I5,
	I6,
	I7,
	I8,
	I9,
	I10,
	I11,
	I12,
	I13,
	I14,
	I15,
}iWord_t;

typedef enum
{
	D0, //lastPuledInsIdPD (read only)
	D1, //resExecutedInsIdPD (read only)
	D2, //vfd speed request
	D3, //vfd speed (read only)
	D4, //vfd speed lim

	D5, //yServo ODO meter L
	D6, //yServo ODO meter H
	D7, //yServo TRIP meter L
	D8, //yServo TRIP meter H
	D9, //yServo set output meter L
	D10, //yServo set output meter H

	D11,
	D12,
	D13,
	D14,
	D15,
}word_t;

typedef enum
{
	F0,
	F1,
	F2,
	F3,
	F4,
	F5,
	F6,
	F7,
	F8,
	F9,
	F10,
	F11,
	F12,
	F13,
	F14,
	F15,
}fWord_t;





#endif /* INC_PLC_DEF_H_ */
