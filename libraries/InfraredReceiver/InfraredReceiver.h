/*
 * IRremote
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.htm http://arcfn.com
 * Edited by Mitra to add new controller SANYO
 *
 * Interrupt code based on NECIRrcv by Joe Knapp
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
 * Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
 *
 * JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
 */

#ifndef _Infrared_Receiver_H_
#define _Infrared_Receiver_H_


#include <Arduino.h>


// len, start_H, start_L, nshort, nlong, data_len, data[data_len]....
#define D_LEN       0
#define D_STARTH    1
#define D_STARTL    2
#define D_SHORT     3
#define D_LONG      4
#define D_DATALEN   5
#define D_DATA      6

#define USECPERTICK 50  // microseconds per clock interrupt tick
#define RAWBUF 256 // Length of raw duration buffer

// Marks tend to be 100us too long, and spaces 100us too short
// when received due to sensor lag.
#define MARK_EXCESS 100

#define __DEBUG     0

// receiver states
#define STATE_IDLE     2
#define STATE_MARK     3
#define STATE_SPACE    4
#define STATE_STOP     5

// information for the interrupt handler
typedef struct {
    uint8_t recvpin;              // pin for IR data from detector
    uint8_t rcvstate;             // state machine
    unsigned int timer;           // state timer, counts 50uS ticks.
    unsigned int rawbuf[RAWBUF];  // raw data
    uint8_t rawlen;               // counter of entries in rawbuf
} irparams_t;

#define MAX_IRPARAMS    2

// IR detector output is active low
#define MARK  0
#define SPACE 1

#define ERR 0

#define _GAP 5000 // Minimum map between transmissions
#define GAP_TICKS (_GAP/USECPERTICK)

// Results returned from the decoder
class decode_results
{
    public:
    volatile unsigned int *rawbuf; // Raw intervals in .5 us ticks
    int rawlen;           // Number of records in rawbuf.
};

// main class for receiving IR
class InfraredReceiver
{
public:

	InfraredReceiver(int revPin);	
	void timerInit(void);
    void Init(void);
    unsigned char *InfraredReceiverRead(void);
    int InfraredReceiverReadCode(void);
	
private:

    void Clear();
	unsigned char IsData();
    int decode(decode_results *results);
    void enableIRIn();
	
    uint8_t IRIndex;  
    
	int _pin;
	unsigned char revData[64];
    decode_results results;
};


#endif
