
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>

#define GROVE_TWO_TX_PIN_NUM	PA2
#define GROVE_TWO_RX_PIN_NUM	PA3

#define MATRIX_N0	PB1
#define MATRIX_N1	PA13
#define MATRIX_N2	PA14
#define MATRIX_N3	PF0
#define MATRIX_N4	PF1

#define MATRIX_P5	PA0
#define MATRIX_P4	PA1
#define MATRIX_P3	PA4
#define MATRIX_P2	PA5
#define MATRIX_P1	PA6
#define MATRIX_P0	PA7

/***************************************************************
6x5
 ***************************************************************/
const uint8_t Font6x5[] = 
{
	0x00, 0x00, 0x00, 0x00, 0x00,//  (space)
	0x00, 0x00, 0x2F, 0x00, 0x00,// !
	0x00, 0x06, 0x00, 0x06, 0x00,// "
	0x14, 0x3E, 0x14, 0x3E, 0x14,// #
	0x02, 0x15, 0x3F, 0x15, 0x08,// $
	0x20, 0x12, 0x08, 0x24, 0x02,// %
	0x1A, 0x25, 0x25, 0x0A, 0x10,// &
	0x00, 0x04, 0x03, 0x00, 0x00,// '
	0x00, 0x1E, 0x33, 0x21, 0x00,// (
	0x00, 0x21, 0x33, 0x1E, 0x00,// )
	0x08, 0x2A, 0x1C, 0x2A, 0x08,// *
	0x00, 0x08, 0x1C, 0x08, 0x00,// +
	0x00, 0x20, 0x38, 0x18, 0x00,// ,
	0x00, 0x08, 0x08, 0x08, 0x00,// -
	0x00, 0x00, 0x30, 0x30, 0x00,// .
	0x00, 0x10, 0x0C, 0x02, 0x00,// /
	0x1E, 0x21, 0x21, 0x1E, 0x00,// 0
	0x00, 0x02, 0x3F, 0x00, 0x00,// 1
	0x00, 0x32, 0x29, 0x26, 0x00,// 2
	0x11, 0x21, 0x25, 0x1B, 0x00,// 3
	0x18, 0x14, 0x12, 0x3F, 0x10,// 4
	0x27, 0x25, 0x25, 0x19, 0x00,// 5
	0x3C, 0x2A, 0x29, 0x39, 0x00,// 6
	0x01, 0x31, 0x09, 0x07, 0x00,// 7
	0x1A, 0x25, 0x25, 0x25, 0x1A,// 8
	0x02, 0x25, 0x15, 0x0D, 0x06,// 9
	0x00, 0x00, 0x14, 0x00, 0x00,// :
	0x00, 0x20, 0x14, 0x00, 0x00,// ;
	0x00, 0x08, 0x14, 0x22, 0x00,// <
	0x00, 0x14, 0x14, 0x14, 0x00,// =
	0x00, 0x22, 0x14, 0x08, 0x00,// >
	0x00, 0x01, 0x2D, 0x02, 0x00,// ?
//	0x06, 0x29, 0x2F, 0x21, 0x1E,// @
	0x1E, 0x21, 0x2F, 0x29, 0x06,// @
	0x3C, 0x0A, 0x09, 0x0A, 0x3C,// A
	0x3F, 0x25, 0x25, 0x1A, 0x00,// B
	0x1E, 0x21, 0x21, 0x21, 0x00,// C
	0x3F, 0x21, 0x21, 0x1E, 0x00,// D
	0x3F, 0x25, 0x25, 0x25, 0x00,// E
	0x3F, 0x05, 0x05, 0x01, 0x00,// F
	0x1E, 0x21, 0x29, 0x39, 0x00,// G
	0x3F, 0x04, 0x04, 0x3F, 0x00,// H
	0x00, 0x21, 0x3F, 0x21, 0x00,// I
	0x10, 0x21, 0x3F, 0x01, 0x00,// J
	0x3F, 0x04, 0x0A, 0x11, 0x20,// K
	0x3F, 0x20, 0x20, 0x20, 0x00,// L
	0x3F, 0x02, 0x04, 0x02, 0x3F,// M
	0x3F, 0x06, 0x0C, 0x18, 0x3F,// N
	0x1E, 0x21, 0x21, 0x21, 0x1E,// O
	0x3F, 0x09, 0x09, 0x09, 0x06,// P
	0x1E, 0x21, 0x29, 0x1E, 0x20,// Q
	0x3F, 0x09, 0x19, 0x29, 0x26,// R
	0x22, 0x25, 0x25, 0x25, 0x19,// S
	0x01, 0x01, 0x3F, 0x01, 0x01,// T
	0x1F, 0x20, 0x20, 0x20, 0x1F,// U
	0x0F, 0x18, 0x30, 0x18, 0x0F,// V
	0x1E, 0x20, 0x1C, 0x20, 0x1E,// W
	0x3B, 0x04, 0x04, 0x3B, 0x00,// X
	0x03, 0x04, 0x3C, 0x04, 0x03,// Y
	0x22, 0x32, 0x2A, 0x26, 0x22,// Z
	0x00, 0x3E, 0x22, 0x00, 0x00,// [
	0x00, 0x02, 0x0C, 0x10, 0x00,// '\'
	0x00, 0x22, 0x3E, 0x00, 0x00,// ]
	0x00, 0x02, 0x01, 0x02, 0x00,// ^
	0x00, 0x10, 0x10, 0x10, 0x00,// _
	0x00, 0x00, 0x03, 0x04, 0x00,// `
	0x1C, 0x22, 0x22, 0x3E, 0x20,// a
	0x3E, 0x28, 0x28, 0x10, 0x00,// b
	0x1C, 0x22, 0x22, 0x22, 0x00,// c
	0x10, 0x28, 0x28, 0x3E, 0x00,// d
	0x1C, 0x2A, 0x2A, 0x2C, 0x00,// e
	0x08, 0x3C, 0x0A, 0x0A, 0x00,// f
	0x04, 0x2A, 0x2A, 0x1E, 0x00,// g
	0x3E, 0x08, 0x38, 0x00, 0x00,// h
	0x00, 0x00, 0x3D, 0x00, 0x00,// i
	0x10, 0x20, 0x3D, 0x00, 0x00,// j
	0x3F, 0x08, 0x14, 0x20, 0x00,// k
	0x00, 0x3E, 0x00, 0x00, 0x00,// l
	0x3E, 0x02, 0x0C, 0x02, 0x3E,// m
	0x3E, 0x02, 0x02, 0x3C, 0x00,// n
	0x1C, 0x22, 0x22, 0x1C, 0x00,// o
	0x3E, 0x0A, 0x0A, 0x04, 0x00,// p
	0x04, 0x0A, 0x0A, 0x3E, 0x00,// q
	0x3E, 0x04, 0x02, 0x02, 0x00,// r
	0x24, 0x2A, 0x2A, 0x12, 0x00,// s
	0x04, 0x1E, 0x24, 0x24, 0x00,// t
	0x1E, 0x20, 0x20, 0x3E, 0x20,// u
	0x0E, 0x10, 0x20, 0x10, 0x0E,// v
	0x1C, 0x20, 0x18, 0x20, 0x1C,// w
	0x24, 0x18, 0x18, 0x24, 0x00,// x
	0x22, 0x14, 0x08, 0x06, 0x00,// y
	0x32, 0x2A, 0x26, 0x22, 0x00,// z
	0x00, 0x00, 0x1C, 0x2A, 0x00,// {
	0x00, 0x00, 0x1E, 0x00, 0x00,// |
	0x00, 0x2A, 0x1C, 0x00, 0x00,// }
	0x08, 0x04, 0x08, 0x04, 0x00,// ~	
};

const uint8_t Icon6x5[] = 
{
    0x04, 0x0e, 0x1c, 0x0e, 0x04,// 0. heart
    0x00, 0x0c, 0x18, 0x0c, 0x00,// 1. small heart
    0x08, 0x10, 0x08, 0x04, 0x02,// 2. yes
    0x22, 0x14, 0x08, 0x14, 0x22,// 3. no
    0x3f, 0x1e, 0x1e, 0x1e, 0x3f,// 4. Grove Zero
    0x08, 0x12, 0x10, 0x12, 0x08,// 5. smile
    0x10, 0x0A, 0x08, 0x0A, 0x10,// 6. sad
    0x00, 0x3A, 0x28, 0x3A, 0x00,// 7. surprise
    0x03, 0x2A, 0x20, 0x2A, 0x03,// 8. mad
    0x00, 0x12, 0x10, 0x12, 0x00,// 9. calm
    0x02, 0x2a, 0x10, 0x2a, 0x02,// 10. zip lip
    0x02, 0x2b, 0x22, 0x2b, 0x02,// 11. hat man
    0x34, 0x26, 0x3f, 0x06, 0x04,// 12. umbrella
    0x00, 0x02, 0x29, 0x06, 0x00,// 13. question mark
    0x00, 0x00, 0x2f, 0x00, 0x00,// 14. exclamation mark
    0x02, 0x3f, 0x10, 0x10, 0x30,// 15. giraffe
    0x0f, 0x1a, 0x3f, 0x10, 0x38,// 16. cat
    0x0c, 0x3e, 0x2f, 0x3e, 0x0c,// 17. mushroom
    0x19, 0x26, 0x34, 0x26, 0x19,// 18. telebubbies
    0x04, 0x3e, 0x07, 0x3e, 0x04,// 19. jellyfish
    0x04, 0x02, 0x3f, 0x02, 0x04,// 20. up
    0x04, 0x0e, 0x15, 0x04, 0x04,// 21. down
    0x08, 0x10, 0x3f, 0x10, 0x08,// 22. left
    0x04, 0x04, 0x15, 0x0e, 0x04,// 23. right
    0x0e, 0x11, 0x15, 0x11, 0x0e,// 24. target
};

#define	CACUL_BAR

#ifdef	CACUL_BAR
const uint8_t Bar6x5[8] = 
{
	0x00, // 0
	0x20, // 3
	0x30, // 6
	0x38, // 9
	0x3C, // 12
	0x3E, // 15
	0x3F, // 18
}; 
#else
const uint8_t Bar6x5[] = 
{
	0x00, 0x00, 0x00, 0x00, 0x00,// 0
	0x00, 0x00, 0x20, 0x00, 0x00,// 1
	0x00, 0x20, 0x20, 0x20, 0x00,// 2
	0x20, 0x20, 0x20, 0x20, 0x20,// 3
	0x20, 0x20, 0x30, 0x20, 0x20,// 4
	0x20, 0x30, 0x30, 0x30, 0x20,// 5
	0x30, 0x30, 0x30, 0x30, 0x30,// 6
	0x30, 0x30, 0x38, 0x30, 0x30,// 7
	0x30, 0x38, 0x38, 0x38, 0x30,// 8
	0x38, 0x38, 0x38, 0x38, 0x38,// 9
	0x38, 0x38, 0x3C, 0x38, 0x38,// 10
	0x38, 0x3C, 0x3C, 0x3C, 0x38,// 11
	0x3C, 0x3C, 0x3C, 0x3C, 0x3C,// 12
	0x3C, 0x3C, 0x3E, 0x3C, 0x3C,// 13
	0x3C, 0x3E, 0x3E, 0x3E, 0x3C,// 14
	0x3E, 0x3E, 0x3E, 0x3E, 0x3E,// 15
	0x3E, 0x3E, 0x3F, 0x3E, 0x3E,// 16
	0x3E, 0x3F, 0x3F, 0x3F, 0x3E,// 17
	0x3F, 0x3F, 0x3F, 0x3F, 0x3F,// 18
}; 
#endif
/***************************************************************
5x6
 ***************************************************************/
const uint8_t Font5x6[] = 
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  (space)
	0x00, 0x00, 0x17, 0x00, 0x00, 0x00, // !
	0x00, 0x03, 0x00, 0x03, 0x00, 0x00, // "
	0x0A, 0x1F, 0x0A, 0x1F, 0x0A, 0x00, // #
	0x02, 0x15, 0x1F, 0x15, 0x08, 0x00, // $
	0x10, 0x09, 0x04, 0x12, 0x01, 0x00, // %
	0x1A, 0x15, 0x15, 0x0A, 0x10, 0x00, // &
	0x00, 0x04, 0x03, 0x00, 0x00, 0x00, // '
	0x00, 0x0E, 0x1B, 0x11, 0x00, 0x00, // (
	0x00, 0x11, 0x1B, 0x0E, 0x00, 0x00, // )
    0x04, 0x15, 0x0E, 0x15, 0x04, 0x00, // *
	0x00, 0x04, 0x0E, 0x04, 0x00, 0x00, // +
	0x00, 0x10, 0x1C, 0x0C, 0x00, 0x00, // ,
	0x00, 0x04, 0x04, 0x04, 0x00, 0x00, // -
	0x00, 0x00, 0x18, 0x18, 0x00, 0x00, // .
	0x00, 0x10, 0x0C, 0x02, 0x00, 0x00, // / 
    0x00, 0x0E, 0x11, 0x11, 0x0E, 0x00, // 0
    0x00, 0x12, 0x1F, 0x10, 0x00, 0x00, // 1
    0x00, 0x19, 0x15, 0x15, 0x12, 0x00, // 2
    0x00, 0x09, 0x11, 0x15, 0x0B, 0x00, // 3
    0x0C, 0x0A, 0x09, 0x1F, 0x08, 0x00, // 4
    0x17, 0x15, 0x15, 0x15, 0x09, 0x00, // 5
    0x08, 0x14, 0x16, 0x15, 0x08, 0x00, // 6
    0x11, 0x09, 0x05, 0x03, 0x01, 0x00, // 7
    0x0A, 0x15, 0x15, 0x15, 0x0A, 0x00, // 8
    0x02, 0x15, 0x0D, 0x05, 0x02, 0x00, // 9
    0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, // :
	0x00, 0x10, 0x0A, 0x00, 0x00, 0x00, // ;
	0x00, 0x04, 0x0A, 0x11, 0x00, 0x00, // <
	0x00, 0x0A, 0x0A, 0x0A, 0x00, 0x00, // =
	0x00, 0x11, 0x0A, 0x04, 0x00, 0x00, // >
	0x00, 0x01, 0x15, 0x02, 0x00, 0x00, // ?
//	0x02, 0x15, 0x15, 0x17, 0x11, 0x0E, // @
	0x0E, 0x11, 0x17, 0x15, 0x15, 0x0A, // @
	0x00, 0x1E, 0x05, 0x05, 0x1E, 0x00, // A
    0x00, 0x1F, 0x15, 0x15, 0x1A, 0x00, // B
    0x00, 0x0E, 0x11, 0x11, 0x11, 0x00, // C
    0x00, 0x1F, 0x11, 0x11, 0x0E, 0x00, // D
    0x00, 0x1F, 0x15, 0x15, 0x15, 0x00, // E
    0x00, 0x1F, 0x05, 0x05, 0x01, 0x00, // F
    0x00, 0x0E, 0x11, 0x15, 0x1D, 0x00, // G
    0x00, 0x1F, 0x04, 0x04, 0x1F, 0x00, // H
    0x11, 0x11, 0x1F, 0x11, 0x11, 0x00, // I
    0x09, 0x11, 0x1F, 0x01, 0x01, 0x00, // J
    0x00, 0x1F, 0x04, 0x0A, 0x11, 0x00, // K
    0x00, 0x1F, 0x10, 0x10, 0x10, 0x00, // L
    0x1F, 0x02, 0x04, 0x02, 0x1F, 0x00, // M
    0x1F, 0x02, 0x04, 0x08, 0x1F, 0x00, // N
    0x00, 0x0E, 0x11, 0x11, 0x0E, 0x00, // O
    0x00, 0x1F, 0x05, 0x05, 0x02, 0x00, // P
    0x00, 0x0E, 0x11, 0x15, 0x0E, 0x10, // Q
    0x00, 0x1F, 0x05, 0x05, 0x0A, 0x10, // R
    0x00, 0x0E, 0x11, 0x15, 0x0E, 0x10, // Q
    0x00, 0x1F, 0x05, 0x05, 0x0A, 0x10, // R
    0x00, 0x12, 0x15, 0x15, 0x09, 0x00, // S
    0x01, 0x01, 0x1F, 0x01, 0x01, 0x00, // T
    0x0F, 0x10, 0x10, 0x10, 0x0F, 0x00, // U
    0x07, 0x08, 0x10, 0x08, 0x07, 0x00, // V
    0x0F, 0x10, 0x0C, 0x10, 0x0F, 0x00, // W
    0x11, 0x0A, 0x04, 0x0A, 0x11, 0x00, // X
    0x01, 0x02, 0x1C, 0x02, 0x01, 0x00, // Y
    0x11, 0x19, 0x15, 0x13, 0x11, 0x00, // Z
    0x00, 0x1F, 0x11, 0x00, 0x00, 0x00, // [
	0x00, 0x02, 0x0C, 0x10, 0x00, 0x00, // '\'
	0x00, 0x11, 0x1F, 0x00, 0x00, 0x00, // ]
    0x00, 0x02, 0x01, 0x02, 0x00, 0x00, // ^
	0x00, 0x10, 0x10, 0x10, 0x00, 0x00, // _
	0x00, 0x03, 0x04, 0x00, 0x00, 0x00, // `
    0x0C, 0x12, 0x12, 0x1E, 0x10, 0x00, // a
    0x00, 0x1F, 0x14, 0x14, 0x08, 0x00, // b
    0x00, 0x0C, 0x12, 0x12, 0x00, 0x00, // c
    0x00, 0x08, 0x14, 0x14, 0x1F, 0x00, // d
    0x00, 0x0E, 0x15, 0x15, 0x16, 0x00, // e
    0x00, 0x04, 0x1E, 0x05, 0x05, 0x00, // f
    0x00, 0x02, 0x15, 0x15, 0x0F, 0x00, // g
    0x00, 0x1F, 0x04, 0x1C, 0x00, 0x00, // h
    0x00, 0x14, 0x1D, 0x10, 0x00, 0x00, // i
    0x00, 0x00, 0x10, 0x1D, 0x00, 0x00, // j
    0x00, 0x1E, 0x04, 0x0A, 0x10, 0x00, // k
    0x00, 0x1E, 0x10, 0x10, 0x00, 0x00, // l
    0x1C, 0x02, 0x0C, 0x02, 0x1C, 0x00, // m
    0x00, 0x1E, 0x02, 0x02, 0x1C, 0x00, // n
    0x00, 0x0C, 0x12, 0x12, 0x0C, 0x00, // o
    0x00, 0x1E, 0x0A, 0x0A, 0x04, 0x00, // p
    0x00, 0x04, 0x0A, 0x0A, 0x1E, 0x00, // q
    0x00, 0x1E, 0x04, 0x02, 0x02, 0x00, // r
    0x00, 0x12, 0x15, 0x0D, 0x00, 0x00, // s
    0x00, 0x02, 0x0F, 0x12, 0x12, 0x00, // t
    0x00, 0x0E, 0x10, 0x10, 0x0E, 0x00, // u
    0x06, 0x08, 0x10, 0x08, 0x06, 0x00, // v
    0x0E, 0x10, 0x08, 0x10, 0x0E, 0x00, // w
    0x12, 0x0C, 0x0C, 0x12, 0x00, 0x00, // x
    0x11, 0x0A, 0x04, 0x03, 0x00, 0x00, // y
    0x12, 0x1A, 0x16, 0x12, 0x00, 0x00, // z
    0x00, 0x00, 0x0E, 0x15, 0x00, 0x00, // {
	0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, // |
	0x00, 0x15, 0x0E, 0x00, 0x00, 0x00, // }
	0x00, 0x04, 0x02, 0x04, 0x02, 0x00, // ~
};

const uint8_t Icon5x6[] = 
{
    0x06, 0x0F, 0x1E, 0x1E, 0x0F, 0x06,// 0. heart
    0x00, 0x06, 0x0C, 0x0C, 0x06, 0x00,// 1. small heart
    0x08, 0x10, 0x08, 0x04, 0x02, 0x00,// 2. yes
    0x11, 0x0A, 0x04, 0x0A, 0x11, 0x00,// 3. no
    0x1F, 0x0E, 0x0E, 0x0E, 0x0E, 0x1F,// 4. Grove Zero
    0x08, 0x12, 0x10, 0x10, 0x12, 0x08,// 5. smile
    0x10, 0x0A, 0x08, 0x08, 0x0A, 0x10,// 6. sad
    0x00, 0x1D, 0x14, 0x14, 0x1D, 0x00,// 7. surprise
    0x03, 0x16, 0x10, 0x10, 0x16, 0x03,// 8. mad
    0x00, 0x12, 0x10, 0x10, 0x12, 0x00,// 9. calm
    0x01, 0x11, 0x08, 0x08, 0x11, 0x01,// 10. zip lip
    0x02, 0x17, 0x12, 0x12, 0x17, 0x02,// 11. hat man
    0x1A, 0x13, 0x1F, 0x03, 0x02, 0x00,// 12. umbrella
    0x02, 0x01, 0x15, 0x05, 0x02, 0x00,// 13. question mark
    0x00, 0x00, 0x17, 0x00, 0x00, 0x00,// 14. exclamation mark
    0x02, 0x1F, 0x08, 0x08, 0x18, 0x00,// 15. giraffe
    0x0F, 0x0A, 0x1F, 0x08, 0x08, 0x1C,// 16. cat
    0x04, 0x1E, 0x17, 0x17, 0x1E, 0x04,// 17. mushroom
    0x0C, 0x13, 0x1A, 0x1A, 0x13, 0x0C,// 18. telebubbies
    0x04, 0x1E, 0x07, 0x07, 0x1E, 0x04,// 19. jellyfish
    0x04, 0x02, 0x1F, 0x02, 0x04, 0x00,// 20. up
    0x04, 0x0E, 0x15, 0x04, 0x04, 0x04,// 21. down
    0x04, 0x08, 0x1F, 0x08, 0x04, 0x00,// 22. left
    0x04, 0x04, 0x04, 0x15, 0x0E, 0x04,// 23. right
    0x0E, 0x11, 0x15, 0x11, 0x0E, 0x00,// 24. target
};

/***************************************************************

 ***************************************************************/
#define DISPLAY_ROTATE_0	  0
#define DISPLAY_ROTATE_90	  1
#define DISPLAY_ROTATE_180    2
#define DISPLAY_ROTATE_270    3

uint8_t displayOrientation = DISPLAY_ROTATE_0;
bool displayPriorityFlag = false;
bool displayOffsetFlalg = false;
int16_t displayOffsetX = 0;
int16_t displayOffsetY = 0;
uint8_t displayModeBackup = 0;
uint8_t displayDataBackup = 0;
uint16_t displayTimeBackup = 0;
uint8_t displayCustomBackup[8];
uint8_t displayBuffBackup[8];
uint8_t displayNumberBackup = 0;
uint8_t displayStringLenghBackup = 0, displayStringLengh = 0;
bool displayStringFlagBackup = false;
uint8_t	ledFlashTimes = 0;

const uint8_t negativePins[5] = {MATRIX_N0, MATRIX_N1, MATRIX_N2, MATRIX_N3, MATRIX_N4};
const uint8_t positivePins[6] = {MATRIX_P0, MATRIX_P1, MATRIX_P2, MATRIX_P3, MATRIX_P4, MATRIX_P5};

uint8_t edgeBitSelector;
uint8_t displayBuff[8];

uint32_t printTimer = 0;
uint32_t printTimerPreviousMillis = 0;
bool printFlag = false;
bool printAlway = false;

bool printMotion = false;
uint8_t stringBuff[32] = {0,};
uint8_t stringLength = 0;
uint8_t stringIndex = 0;

void led5x6PrintOff(void);
void led5x6Print(void);
void led5x6PrintAscii(uint8_t ascii, uint16_t displayTime);
void led5x6PrintIcon(uint8_t picNum, uint16_t displayTime);
void led5x6PrintBar(uint8_t barNum, uint16_t displayTime);
void led5x6PrintString(uint8_t len);
void led5x6PrintData(uint8_t *buffer, uint16_t displayTime);
void led5x6PrintNumber(int32_t num, uint16_t displayTime);
void led5x6SetOffset(void);

/***************************************************************

 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x07
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x8002

#define I2C_DEF_ADDR_FLASH_LOC	0x00
#define I2C_CUR_ADDR_FLASH_LOC	0x01

#define I2C_CMD_GET_DEV_ID		0x00 // 
#define I2C_CMD_DISP_BAR		0x01 //
#define I2C_CMD_DISP_PHIZ		0x02 //
#define I2C_CMD_DISP_NUM		0x03 //
#define I2C_CMD_DISP_STR		0x04 //
#define I2C_CMD_DISP_DATA		0x05 //
#define I2C_CMD_DISP_OFF		0x06 //
#define I2C_CMD_LED_ON			0xb0 // 
#define I2C_CMD_LED_OFF			0xb1 // 
#define I2C_CMD_AUTO_SLEEP_ON	0xb2 // 
#define I2C_CMD_AUTO_SLEEP_OFF	0xb3 // 
#define I2C_CMD_DISP_OFFSET     0xb5 // 
#define I2C_CMD_SET_ADDR		0xc0 //
#define I2C_CMD_RST_ADDR		0xc1 // 
#define I2C_CMD_TEST_TX_RX_ON   0xe0 // 
#define I2C_CMD_TEST_TX_RX_OFF  0xe1 // 
#define I2C_CMD_TEST_GET_VER    0xe2 // 
#define I2C_CMD_JUMP_TO_BOOT	0xf0 // 
#define I2C_CMD_GET_DEVICE_UID  0xf1 // 
#define I2C_CMD_NULL			0xff // 

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
uint8_t commandReceive = I2C_CMD_NULL;


#ifdef BLE_SUPPORT

#define I2C_CMD_BLE_DISP_OFFSET		0x90
#define I2C_CMD_LOW_PRIORITY		0x90

uint8_t Core_mode = 0;
uint32_t StartMillis = 0;

typedef struct
{
	uint8_t Datalen;
	uint8_t type;
	uint8_t Address;
	uint8_t Option;
}packet_header_t;

typedef struct
{
	uint8_t 	flag;
	uint8_t		time[2];
	uint8_t 	orient;
	uint8_t		str[8];
}packet_diplay_str;

typedef struct
{
	uint8_t 	pid[2];
	uint8_t 	chipid;
	uint8_t 	Newaddress;
	uint8_t 	option[5];
}packet_got_atr;

typedef struct
{
	packet_header_t	Header;
	union 
	{
		packet_diplay_str		str;
		packet_got_atr			atr;
		uint8_t Option[MAINBOARD_BLE_I2C_DATALEN-sizeof(packet_header_t)];
	}commands;

}packet_thld_t; // 8 bytes

union
{
    packet_thld_t data;
    uint8_t bytes[MAINBOARD_BLE_I2C_DATALEN]; 
}commandOption;

typedef struct
{
	packet_header_t	Header;
	uint8_t		data[2];
}packet_raw;

typedef struct
{
	packet_header_t	Header;
	uint8_t		Event;
}packet_event;

typedef struct
{
	packet_header_t	Header;
	uint8_t 	pid[2];
	uint8_t 	chipid;
	uint8_t 	hwaddress;
	uint8_t 	version[3];
	uint8_t 	option[2];
}packet_atr;

union
{
	packet_atr		atr;
	packet_event	event;
	packet_raw		raw_data;
    uint8_t 		bytes[MAINBOARD_BLE_I2C_DATALEN]; 
}InstructionOption;

uint8_t *ptr2 = (uint8_t *)&InstructionOption;

#endif

typedef struct
{
	uint16_t deviceVID;
	uint16_t devicePID;
	uint32_t deviceEvent;
}packet_01_t; // 8 bytes

union
{
    packet_01_t data;
    uint8_t bytes[8]; 
}packet_01_data;

uint8_t *ptr1 = (uint8_t *)&packet_01_data;

void requestEvent(void);
void receiveEvent(int howMany);

/***************************************************************

 ***************************************************************/
LowPower nrgSave;

#define AUTO_SLEEP_TIMEOUT	2000

uint32_t autoSleepPreviousMillis = 0, PreMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

#define LED_FLASH_TIME	250

bool ledFlashCommand = false;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

bool testFlag = false;
char *versions = "V20";
uint16_t NodeVersion = 0x6101;

uint32_t intStart = 0;
uint32_t intEnd = 0;
uint8_t data[5] = {0,};
uint8_t ContinueNumber = 0;
/***************************************************************

 ***************************************************************/
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/***************************************************************

 ***************************************************************/
void setup()
{
	uint8_t i2cDefaultAddr = Flash.read8(I2C_DEF_ADDR_FLASH_LOC); 
	uint8_t i2cCurrentAddr = Flash.read8(I2C_CUR_ADDR_FLASH_LOC);
    
    uint8_t *ptr3 = (uint8_t *)Flash.getChipUniqueID();
    for(uint8_t i = 0; i < 12; i ++)chipId[i] = *(ptr3 + i);
	
	srand((uint32_t)(chipId[0]+chipId[1]));
		
	if(i2cDefaultAddr == 0xff)Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	if(i2cCurrentAddr == 0xff)Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	else deviceI2CAddress = i2cCurrentAddr;
	
	packet_01_data.data.deviceVID = DEVICE_VID;
	packet_01_data.data.devicePID = DEVICE_PID;
	packet_01_data.data.deviceEvent = 0;
	
	nrgSave.begin(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance
#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif
	
	for(uint8_t i = 0;i < 5;i ++)pinMode(negativePins[i], OUTPUT);
	for(uint8_t i = 0;i < 6;i ++)pinMode(positivePins[i], OUTPUT);
	led5x6PrintOff();
	
	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);
	
	wwdg.begin();
}

void loop()
{	
	uint8_t RAND;
	uint32_t CurrentMillis = millis();
			
	if(Wire.isbusidle())PreMillis = CurrentMillis;
	if ((CurrentMillis - PreMillis) > 20)
	{
		Wire.reset();
		Wire.begin(deviceI2CAddress);
		Wire.onReceive(receiveEvent);
		Wire.onRequest(requestEvent);
		PreMillis = CurrentMillis;
	}
	
	if(printMotion == false)
	{
		if(printTimer > 0)
		{

			if(CurrentMillis - printTimerPreviousMillis >= printTimer)
			{
				printFlag = false;
				displayPriorityFlag = false;
				printTimer = 0;
				led5x6PrintOff();
			}
		}	
	}
	else
	{
		if(CurrentMillis - printTimerPreviousMillis >= printTimer)
		{
			uint16_t index;
			printTimerPreviousMillis = CurrentMillis;
            
            if(displayOrientation == DISPLAY_ROTATE_0 || displayOrientation == DISPLAY_ROTATE_180)
            {
                if((stringIndex % 5) == 0)
                {
                    for(uint8_t j = 0;j < 5;j ++)
                    {
						if(stringBuff[stringIndex / 5] < 32)stringBuff[stringIndex / 5] = 32;
                        index = (stringBuff[stringIndex / 5] - 32) * 5 + j; // get the 1/5 code address for each asc;
                        displayBuff[j] = Font6x5[index]; // get the code 
                    }
                }
                else
                {
                    uint8_t temp;
                    if(stringIndex < (stringLength - 5))
                    {
						if(stringBuff[stringIndex / 5 + 1] < 32)stringBuff[stringIndex / 5 + 1] = 32;
                        index = (stringBuff[stringIndex / 5 + 1] - 32) * 5 + (stringIndex % 5) - 1;
                        temp = Font6x5[index];
                    }
                    else temp = 0x00;
                    displayBuff[0] = displayBuff[1];
                    displayBuff[1] = displayBuff[2];
                    displayBuff[2] = displayBuff[3];
                    displayBuff[3] = displayBuff[4];
                    displayBuff[4] = temp;
                }
            }
            else if(displayOrientation == DISPLAY_ROTATE_90 || displayOrientation == DISPLAY_ROTATE_270)
            {
                if((stringIndex % 6) == 0)
                {
                    for(uint8_t j = 0;j < 6;j ++)
                    {
						if(stringBuff[stringIndex / 6] < 32)stringBuff[stringIndex / 6] = 32;
                        index = (stringBuff[stringIndex / 6] - 32) * 6 + j; // get the 1/6 code address for each asc;
                        displayBuff[j] = Font5x6[index]; // get the code 
                    }
                }
                else
                {
                    uint8_t temp;
                    if(stringIndex < (stringLength - 6))
                    {
						if(stringBuff[stringIndex / 6 + 1] < 32)stringBuff[stringIndex / 6 + 1] = 32;
                        index = (stringBuff[stringIndex / 6 + 1] - 32) * 6 + (stringIndex % 6) - 1;
                        temp = Font5x6[index];
                    }
                    else temp = 0x00;
                    displayBuff[0] = displayBuff[1];
                    displayBuff[1] = displayBuff[2];
                    displayBuff[2] = displayBuff[3];
                    displayBuff[3] = displayBuff[4];
                    displayBuff[4] = displayBuff[5];
                    displayBuff[5] = temp;
                }
            }
			
			stringIndex ++;
			if(stringIndex >= stringLength) // display finish, 
			{
				if(printAlway == false)
				{	
					printFlag = false;
					printMotion = false;
					displayPriorityFlag = false;
					stringLength = 0;
					stringIndex = 0;
					printTimer = 0;
					led5x6PrintOff();
				}
				else
				{
					stringIndex = 0;
				}
                
                for(uint8_t i = 0; i < 8; i ++)displayBuff[i] = 0;	
			}	
		}
	}
	
	if(ledFlashTimes < 18)
	{
		if(CurrentMillis - ledFlashPreviousMillis >= 50)
		{
			ledFlashPreviousMillis = CurrentMillis;
			if(ledFlashTimes < 5)
			{
				data[ledFlashTimes] = 1;
			}else if (ledFlashTimes < 10)
			{
				data[4] |= (0x01<<(ledFlashTimes-4));
			}else if (ledFlashTimes < 14)
			{
				data[13-ledFlashTimes] |= 0x20;
			}else if (ledFlashTimes < 18)
			{
				data[0] |= 0x20>>(ledFlashTimes-13);
			}
			led5x6PrintData(data, 0);
			ledFlashTimes++;
		}
	}else if(ledFlashTimes < 22)
	{
		if(CurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
		{
			ledFlashPreviousMillis = CurrentMillis;
			if(ledFlashStatus)
			{
				for(uint8_t i = 0; i < 5; i ++)data[i] = 0xff;
				led5x6PrintData(data, LED_FLASH_TIME);
			}
			else
			{
				for(uint8_t i = 0; i < 5; i ++)data[i] = 0;
				led5x6PrintData(data, LED_FLASH_TIME);
				ledFlashTimes++;
			}
			ledFlashStatus = !ledFlashStatus;
		}
	}
	if(ledFlashCommand)
	{
		if(CurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
		{
//			uint8_t data[5];
			ledFlashPreviousMillis = CurrentMillis;
			if(ledFlashStatus)
			{
				for(uint8_t i = 0; i < 5; i ++)data[i] = 0xff;
				led5x6PrintData(data, LED_FLASH_TIME);
			}
			else
			{
				for(uint8_t i = 0; i < 5; i ++)data[i] = 0;
				led5x6PrintData(data, LED_FLASH_TIME);
			}
			ledFlashStatus = !ledFlashStatus;
		}
	}

#ifdef BLE_SUPPORT
	if (Core_mode == 0)
	{// ATR
		if(CurrentMillis - StartMillis >= (I2C_CMD_SYS_READY_TIME+deviceI2CAddress*2))
		{
#if 1
			Core_mode = CORE_BLE_MODE;
#else
			commandReceive = I2C_CMD_NOTIFY_ATR;
			StartMillis = CurrentMillis;
#endif
		}
	}
	if(Core_mode == CORE_BLE_MODE)
	{// Message process.

		switch(commandReceive)
		{
			case I2C_CMD_NOTIFY_ATR:
				InstructionOption.atr.Header.type	= I2C_CMD_ATR;
				InstructionOption.atr.Header.Address= deviceI2CAddress;
				InstructionOption.atr.pid[0]		= DEVICE_PID&0xff;
				InstructionOption.atr.pid[1]		= (DEVICE_PID>>8 ) & 0xff;
				InstructionOption.atr.chipid		= chipId[0];
				InstructionOption.atr.hwaddress		= DEVICE_I2C_ADDRESS;
				InstructionOption.atr.version[0]	= versions[0];
				InstructionOption.atr.version[1]	= versions[1];
				InstructionOption.atr.version[2]	= versions[2];
				InstructionOption.atr.option[0] 	= NodeVersion&0xFF;
				InstructionOption.atr.option[1] 	= (NodeVersion>>8)&0xFF;
				InstructionOption.atr.Header.Datalen = sizeof(packet_atr);
				Wire.MasterGPIOTransmission(ptr2, sizeof(packet_atr));
				commandReceive = I2C_CMD_NULL;

				break;
			case I2C_CMD_ATR:
				if ((commandOption.data.commands.atr.pid[0] == (DEVICE_PID&0xFF)) && 
					(commandOption.data.commands.atr.pid[1] == ((DEVICE_PID>>8)&0xFF)) && 
					(commandOption.data.commands.atr.chipid == chipId[0]))
				{// It's for current device
					if(commandOption.data.commands.atr.Newaddress != deviceI2CAddress)
					{
						deviceI2CAddress = commandOption.data.commands.atr.Newaddress;
						Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
						Wire.begin(deviceI2CAddress);
					}					
				}
				commandReceive = I2C_CMD_NULL;
				break;

			default:
			break;
		}
	}else 
	{
#endif

	if(commandReceive == I2C_CMD_SET_ADDR) // change i2c address
	{
		commandReceive = I2C_CMD_NULL;
		Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
		Wire.begin(deviceI2CAddress);
	}
	else if(commandReceive == I2C_CMD_RST_ADDR) // reset i2c address
	{
		commandReceive = I2C_CMD_NULL;
		deviceI2CAddress = Flash.read8(I2C_DEF_ADDR_FLASH_LOC);
		Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
		Wire.begin(deviceI2CAddress);
	}
#ifdef BLE_SUPPORT
	}
#endif

	if(printFlag)led5x6Print();
	else // no display, go to sleep
	{
		if(autoSleepFlag)
		{
			uint32_t autoSleepCurrentMillis = millis();
			if((autoSleepCurrentMillis - autoSleepPreviousMillis) > AUTO_SLEEP_TIMEOUT)
			{
				autoSleepPreviousMillis = autoSleepCurrentMillis;
				
				wwdg.end();
				Wire.end();
				pinMode(PA9, INPUT_PULLUP);
				pinMode(PA10, INPUT_PULLUP);
				
				nrgSave.standby();
				
				Wire.begin(deviceI2CAddress);
				Wire.onReceive(receiveEvent);
				Wire.onRequest(requestEvent);
				wwdg.begin();
			}
		}
	}
	
    if(testFlag)
    {
        wwdg.end();
        pinMode(GROVE_TWO_TX_PIN_NUM, OUTPUT);
        pinMode(GROVE_TWO_RX_PIN_NUM, OUTPUT);
        
        while(1)
        {           
            digitalWrite(GROVE_TWO_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_TWO_RX_PIN_NUM, HIGH);
            delay(1);
            digitalWrite(GROVE_TWO_TX_PIN_NUM, LOW);
            delay(1);
            
            digitalWrite(GROVE_TWO_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_TWO_RX_PIN_NUM, LOW);
            delay(1);
            digitalWrite(GROVE_TWO_TX_PIN_NUM, LOW);
            delay(1);
            
            if(testFlag == false)break;
        }
        
        wwdg.begin();
        attachInterrupt(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE, INPUT_PULLUP);	
    }
    
    wwdg.reset();
}

void dummy(void)
{
	autoSleepPreviousMillis = millis();
    
    if(digitalRead(GROVE_TWO_RX_PIN_NUM) == LOW)intStart = autoSleepPreviousMillis;
    else 
    {
        intEnd = autoSleepPreviousMillis;
        if((intEnd - intStart) > 20)delay(500);
        else intStart = intEnd;
    }
}

void receiveEvent(int howMany)
{
	uint8_t i = 0, j = 0, receiveBuffer[MAINBOARD_BLE_I2C_DATALEN*2] = {0,};
	// autoSleepPreviousMillis = millis();
	
	while(Wire.available())
	{
		receiveBuffer[i ++] = Wire.read();
		if(i >= MAINBOARD_BLE_I2C_DATALEN*2)i = 0;
	}
#ifdef BLE_SUPPORT
	if ((receiveBuffer[0] >= MAINBOARD_BLE_COMMAND_LOW) && 
		(receiveBuffer[0] <  MAINBOARD_BLE_COMMAND_HIGH))
	{// BLE command: len,cmd,opt,...
		Core_mode = CORE_BLE_MODE;
		memcpy(commandOption.bytes, receiveBuffer, i);
		commandOption.data.Header.Datalen -= MAINBOARD_BLE_COMMAND_LOW;
		if (i != commandOption.data.Header.Datalen)
		{// Bus error!!!
			return;
		}
		commandReceive = commandOption.data.Header.type;
		if((commandReceive & 0xF0) == I2C_CMD_LOW_PRIORITY)
		{// Low priority.
			if (displayPriorityFlag)
			{	// High priority task is working.
				commandReceive = I2C_CMD_NULL;
				return;
			}
			commandReceive &= 0x0F;
		}else if (commandReceive < 0x0A)
		{
			displayPriorityFlag = true;	// High priority
		}
		for(j=0;j<(i-sizeof(packet_header_t));j++)
		{
			receiveBuffer[j+1] = receiveBuffer[j+sizeof(packet_header_t)];
		}
		i -= (sizeof(packet_header_t)-1);
		if (commandReceive == (I2C_CMD_BLE_DISP_OFFSET&0x0F))
		{
			commandReceive = I2C_CMD_DISP_OFFSET;
		}
	}else
	{
		Core_mode = CORE_ATMEL_MODE;
#endif
	commandReceive = receiveBuffer[0];
#ifdef BLE_SUPPORT
	}
#endif
	ledFlashTimes = 100;
	
	switch(commandReceive)
	{
		case I2C_CMD_DISP_BAR:
			ledFlashCommand = false;
			ledFlashStatus = false;
            
            displayOrientation = DISPLAY_ROTATE_0;
            displayOffsetX = 0;
            displayOffsetY = 0;
            
			led5x6PrintBar(receiveBuffer[1], receiveBuffer[2] + receiveBuffer[3] * 256);
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_DISP_PHIZ:
			ledFlashCommand = false;
			ledFlashStatus = false;
            
            displayOrientation = receiveBuffer[4];
            displayOffsetX = 0;
            displayOffsetY = 0;
            
			led5x6PrintIcon(receiveBuffer[1], receiveBuffer[2] + receiveBuffer[3] * 256);
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_DISP_NUM:
			ledFlashCommand = false;
			ledFlashStatus = false;
            
            displayOrientation = receiveBuffer[5];
            displayOffsetX = 0;
            displayOffsetY = 0;
            displayModeBackup = 0xff;
            
			led5x6PrintNumber((int16_t)(receiveBuffer[1] + receiveBuffer[2] * 256), receiveBuffer[3] + receiveBuffer[4] * 256);
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_DISP_STR:
			ledFlashCommand = false;
			ledFlashStatus = false;
			if (Core_mode == CORE_ATMEL_MODE)
			{
				displayStringFlagBackup = receiveBuffer[1];
				
				displayOrientation = receiveBuffer[4];
				printAlway = receiveBuffer[1];
				
				displayStringLengh = i - 5;
				displayTimeBackup  = receiveBuffer[2] + receiveBuffer[3] * 256;
				if(displayTimeBackup == 0)
				{
					displayTimeBackup = displayStringLengh*1000;
				}
				if(receiveBuffer[6])
				{
					memcpy(stringBuff, receiveBuffer + 5, i-5);
					displayStringLenghBackup = i - 5;
					stringBuff[displayStringLenghBackup] = 0;
					led5x6PrintString(displayStringLenghBackup);
				}else{
					led5x6PrintAscii(receiveBuffer[5], receiveBuffer[2] + receiveBuffer[3] * 256);
				}
			}else
			{
				displayStringFlagBackup = receiveBuffer[1];
				
				displayOrientation = receiveBuffer[4];
				printAlway = receiveBuffer[1];
				
				displayStringLengh = receiveBuffer[5];
				if (displayStringLengh > 32)displayStringLengh = 32;
				displayTimeBackup  = receiveBuffer[2] + receiveBuffer[3] * 256;
				if(displayTimeBackup == 0)
				{
					displayTimeBackup = displayStringLengh*1000;
				}
				if(displayStringLengh > 1)
				{
					
					memcpy(stringBuff, receiveBuffer + 6, i-6);
					displayStringLenghBackup = i - 6;
					if(displayStringLengh == displayStringLenghBackup)
					{
						stringBuff[displayStringLenghBackup] = 0;
						led5x6PrintString(displayStringLenghBackup);
					}else{
						printMotion = false;
						printTimer  = 1;
						ContinueNumber = 0;
					}
				}else{
					led5x6PrintAscii(receiveBuffer[6], receiveBuffer[2] + receiveBuffer[3] * 256);
				}
			}
			commandReceive = I2C_CMD_NULL;
			break;
    	case I2C_CMD_CONTINUE_DATA:
			if (commandOption.data.Header.Option == ContinueNumber)
			{// Same packet.
				break;
			}
			ContinueNumber ++;
			displayModeBackup = I2C_CMD_DISP_STR;
			if((displayStringLenghBackup+i-1) > displayStringLengh)
			{
				memcpy(stringBuff+displayStringLenghBackup, receiveBuffer + 1, displayStringLengh-displayStringLenghBackup);
				displayStringLenghBackup = displayStringLengh;
			}else
			{
				memcpy(stringBuff+displayStringLenghBackup, receiveBuffer + 1, i-1);
				displayStringLenghBackup += (i-1);
			}				
			
			if(displayStringLengh <= displayStringLenghBackup)
			{
				stringBuff[displayStringLenghBackup] = 0;
				led5x6PrintString(displayStringLenghBackup);
			}
			commandReceive = I2C_CMD_NULL;
			
			break;
		case I2C_CMD_DISP_DATA:
			ledFlashCommand = false;
			ledFlashStatus = false;
            
            displayOrientation = DISPLAY_ROTATE_0;
            displayOffsetX = 0;
            displayOffsetY = 0;
            
			led5x6PrintData(receiveBuffer + 3, receiveBuffer[1] + receiveBuffer[2] * 256);
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_DISP_OFF:
			ledFlashCommand = false;
			ledFlashStatus = false;
			printFlag = false;
			displayPriorityFlag = false;
			printTimer = 0;
			if(printMotion)printMotion = false;
			commandReceive = I2C_CMD_NULL;
			for(uint8_t i = 0; i < 6; i ++)displayBuff[i] = 0;
            
            displayOrientation = DISPLAY_ROTATE_0;
            displayOffsetX = 0;
            displayOffsetY = 0;
		break;
		
		case I2C_CMD_LED_ON:
			ledFlashCommand = true;
			displayPriorityFlag = false;
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_LED_OFF:
			ledFlashCommand = false;
			ledFlashStatus = false;
            printFlag = false;
			displayPriorityFlag = false;
			printTimer = 0;
			led5x6PrintOff();
			commandReceive = I2C_CMD_NULL;

            displayOrientation = DISPLAY_ROTATE_0;
            displayOffsetX = 0;
            displayOffsetY = 0;
		break;
		
		case I2C_CMD_AUTO_SLEEP_ON:
			autoSleepFlag = true;
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_AUTO_SLEEP_OFF:
			autoSleepFlag = false;
			commandReceive = I2C_CMD_NULL;
		break;
        
        case I2C_CMD_DISP_OFFSET:
        {
            int16_t offset = (int16_t)(receiveBuffer[2] + receiveBuffer[3] * 256);
            
            displayOffsetFlalg = receiveBuffer[1];

            if(displayOffsetFlalg == false && displayOrientation == DISPLAY_ROTATE_0)
                displayOffsetX = displayOffsetX - offset;
            
            else if(displayOffsetFlalg == false && displayOrientation == DISPLAY_ROTATE_90)
            {
                displayOffsetFlalg = true;
                displayOffsetY = displayOffsetY + offset;
            }
            
            else if(displayOffsetFlalg == false && displayOrientation == DISPLAY_ROTATE_180)
                displayOffsetX = displayOffsetX + offset;
            
            else if(displayOffsetFlalg == false && displayOrientation == DISPLAY_ROTATE_270)
            {
                displayOffsetFlalg = true;
                displayOffsetY = displayOffsetY - offset;
            }
            
            else if(displayOffsetFlalg == true && displayOrientation == DISPLAY_ROTATE_0)
                displayOffsetY = displayOffsetY + offset;
            
            else if(displayOffsetFlalg == true && displayOrientation == DISPLAY_ROTATE_90)
            {
                displayOffsetFlalg = false;
                displayOffsetX = displayOffsetX + offset;
            }
            
            else if(displayOffsetFlalg == true && displayOrientation == DISPLAY_ROTATE_180)
                displayOffsetY = displayOffsetY - offset;
            
            else if(displayOffsetFlalg == true && displayOrientation == DISPLAY_ROTATE_270)
            {
                displayOffsetFlalg = false;
                displayOffsetX = displayOffsetX - offset;
            }

            if(displayOffsetX > 6)displayOffsetX = 6;
            else if(displayOffsetX < -6)displayOffsetX = -6;
            
            if(displayOffsetY > 6)displayOffsetY = 6;
            else if(displayOffsetY < -6)displayOffsetY = -6;
            
            led5x6SetOffset();
            commandReceive = I2C_CMD_NULL;
        }
        break;
		
		case I2C_CMD_SET_ADDR:
			deviceI2CAddress = receiveBuffer[1];
		break;
        
        case I2C_CMD_TEST_TX_RX_ON:
            testFlag = true;
			commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_TEST_TX_RX_OFF:
            testFlag = false;
			commandReceive = I2C_CMD_NULL;
        break;
		
		case I2C_CMD_JUMP_TO_BOOT:
			commandReceive = I2C_CMD_NULL;
			jumpToBootloader();
		break;
		
		default:
		break;
	}
}

void requestEvent(void)
{
	// autoSleepPreviousMillis = millis();
	
#ifdef BLE_SUPPORT
	Core_mode = CORE_ATMEL_MODE;
#endif
	ledFlashTimes = 100;
	switch(commandReceive)
	{
		case I2C_CMD_GET_DEV_ID:
			Wire.write(ptr1, 4);
			commandReceive = I2C_CMD_NULL;
		break;
        
                
        case I2C_CMD_TEST_GET_VER:
            Wire.write(versions, 3);
			commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_GET_DEVICE_UID:
            Wire.write(chipId, 12);
            commandReceive = I2C_CMD_NULL;
        break;
		
		default:
		break;
	}
}

/***************************************************************

 ***************************************************************/
void led5x6PrintOff(void)
{
	for(uint8_t i = 0; i < 5; i ++)digitalWrite(negativePins[i], HIGH);
	for(uint8_t i = 0; i < 6; i ++)digitalWrite(positivePins[i], LOW);
}

void ledOffset(int8_t OrientX, int8_t OrientY)
{
	int8_t i, offset;
    
    offset = displayOffsetX;
        
    if(offset >= 0)
    {
        if(offset > OrientX)offset = OrientX;

        for(i = 0;i < (OrientX - offset);i ++)
            displayBuff[i] = displayBuffBackup[i + offset];
        
        for(i = (OrientX - offset); i < OrientX; i ++)displayBuff[i] = 0;
    }
    else 
    {
        offset = offset * (-1);

        if(offset > OrientX)offset = OrientX;
        
        for(i = 0; i < offset; i ++)displayBuff[i] = 0;
        for(i = offset; i < OrientX; i ++)
            displayBuff[i] = displayBuffBackup[i - offset];
    }
        
    for(i = 0; i < OrientX; i ++)
        displayBuffBackup[i] = displayBuff[i];  
            
    offset = displayOffsetY;
        
    if(offset >= 0)
    {
        if(offset > OrientY)offset = OrientY;
        
        for(i = 0; i < OrientX; i ++)
            displayBuff[i] = displayBuffBackup[i] >> offset;
    }
    else
    {
        offset = offset * (-1);
        
        if(offset > OrientY)offset = OrientY;

        for(i = 0; i < OrientX; i ++)
            displayBuff[i] = displayBuffBackup[i] << offset;
    }
}

void led5x6Print(void)
{
	uint8_t row, i, j;
	uint8_t bitSelect;
	uint16_t temp; // get the 1/5 code address for each asc;
	
    if(displayOrientation == DISPLAY_ROTATE_0 || displayOrientation == DISPLAY_ROTATE_180)
    {
        for(row = 0;row < 5;row ++)
        {
            if(displayOrientation == DISPLAY_ROTATE_0)
            {
                bitSelect = 0;
                edgeBitSelector = 0x20; // 0x20 = 0b0010 0000,select bit 5 of display control, 
                                        // byte that is read from the array Font5x6						              
            }
            else if(displayOrientation == DISPLAY_ROTATE_180)
            {
                bitSelect = 4;
                edgeBitSelector = 0x01; // always select the LSB of the display control, 
                                        // byte that is read from the array Font5x7
            }
 
            for(i = 0;i < 5;i ++) // 5 bytes for each ASC Code;
            {
                temp = displayBuff[i]; // get the code 

                led5x6PrintOff(); //needed
                
                if(displayOrientation == DISPLAY_ROTATE_0)digitalWrite(negativePins[bitSelect ++], 0);
                else if(displayOrientation == DISPLAY_ROTATE_180)digitalWrite(negativePins[bitSelect --], 0);
                
                for(j = 0;j < 6;j ++)
                {            
                    if(temp & edgeBitSelector) 
                    {
                        digitalWrite(positivePins[j], 1);
                        delayMicroseconds(2);
                        digitalWrite(positivePins[j], 0);
                    }
                    else digitalWrite(positivePins[j], 0);
                    
                    if(displayOrientation == DISPLAY_ROTATE_0)temp <<= 1;
                    else if(displayOrientation == DISPLAY_ROTATE_180)temp >>= 1;
                }
            }
        }
    }
    else if(displayOrientation == DISPLAY_ROTATE_90 || displayOrientation == DISPLAY_ROTATE_270)
    {
        for(row = 0;row < 6;row ++)
        {
            if(displayOrientation == DISPLAY_ROTATE_90)
            {
                bitSelect = 0;
                edgeBitSelector = 0x10; // 0x10 = 0b0001 0000,select bit 4 of display control, 
                                        // byte that is read from the array Font5x6						              
            }
            else if(displayOrientation == DISPLAY_ROTATE_270)
            {
                bitSelect = 5;
                edgeBitSelector = 0x01; // always select the LSB of the display control, 
                                        // byte that is read from the array Font5x7
            }
 
            for(i = 0;i < 6;i ++) // 6 bytes for each ASC Code;
            {
                temp = displayBuff[i]; // get the code 

                led5x6PrintOff(); //needed
                
                if(displayOrientation == DISPLAY_ROTATE_90)digitalWrite(positivePins[5 - (bitSelect ++)], 1);
                else if(displayOrientation == DISPLAY_ROTATE_270)digitalWrite(positivePins[5 - (bitSelect --)], 1);
                
                for(j = 0;j < 5;j ++)
                {            
                    if(temp & edgeBitSelector) 
                    {
                        digitalWrite(negativePins[j], 0);
                        delayMicroseconds(2);
                        digitalWrite(negativePins[j], 1);
                    }
                    else digitalWrite(negativePins[j], 1);
                    
                    if(displayOrientation == DISPLAY_ROTATE_90)temp <<= 1;
                    else if(displayOrientation == DISPLAY_ROTATE_270)temp >>= 1;
                }
            }
        }
    }
}

void led5x6PrintAscii(uint8_t ascii, uint16_t displayTime)
{
	uint16_t i;
	uint16_t temp;
    
    if(displayOrientation == DISPLAY_ROTATE_0 || displayOrientation == DISPLAY_ROTATE_180)
    {
        for(i = 0; i < 5; i ++)
		{
			if (ascii < 32)ascii = 32;
            displayBuffBackup[i] = Font6x5[(ascii - 32) * 5 + i];            
		}
        
        ledOffset(5, 6);
    }
    else if(displayOrientation == DISPLAY_ROTATE_90 || displayOrientation == DISPLAY_ROTATE_270)
    {
        for(i = 0; i < 6; i ++)
		{
			if (ascii < 32)ascii = 32;
            displayBuffBackup[i] = Font5x6[(ascii - 32) * 6 + i];   
		}
        
        ledOffset(6, 5);
    }
    
	printFlag = true;
	printMotion = false;
	printTimer = displayTime;
	printTimerPreviousMillis = millis();
}

void led5x6PrintIcon(uint8_t picNum, uint16_t displayTime)
{
	uint16_t i;
    int8_t offset;

	if(picNum > 24)return;
    
    displayModeBackup = I2C_CMD_DISP_PHIZ;
    displayDataBackup = picNum;
    displayTimeBackup = displayTime;
    
    if(displayOrientation == DISPLAY_ROTATE_0 || displayOrientation == DISPLAY_ROTATE_180)
    {
        for(i = 0; i < 5; i ++)
            displayBuffBackup[i] = Icon6x5[i + picNum * 5];
        
        ledOffset(5, 6);
    }
	else if(displayOrientation == DISPLAY_ROTATE_90 || displayOrientation == DISPLAY_ROTATE_270)
    {     
        for(i = 0; i < 6; i ++)
            displayBuffBackup[i] = Icon5x6[i + picNum * 6];
        
        ledOffset(6, 5);
    }
	
	printFlag = true;
	printMotion = false;
	printTimer = displayTime;
	printTimerPreviousMillis = millis();
}

void led5x6PrintBar(uint8_t barNum, uint16_t displayTime)
{
	uint16_t i;
    int8_t offset;

	if(barNum > 18)barNum = 18;
    
    displayModeBackup = I2C_CMD_DISP_BAR;
    displayDataBackup = barNum;
    displayTimeBackup = displayTime;

    if(displayOrientation == DISPLAY_ROTATE_0 || displayOrientation == DISPLAY_ROTATE_180)
    {
#ifdef	CACUL_BAR
        displayBuffBackup[0] = Bar6x5[barNum/3];
        displayBuffBackup[1] = Bar6x5[(barNum+1)/3];
        displayBuffBackup[2] = Bar6x5[(barNum+2)/3];
        displayBuffBackup[3] = Bar6x5[(barNum+1)/3];
        displayBuffBackup[4] = Bar6x5[barNum/3];
#else
	for(i = 0; i < 5; i ++)
            displayBuffBackup[i] = Bar6x5[i + barNum * 5];
#endif    
        ledOffset(5, 6);
    }
		
	printFlag = true;
	printMotion = false;
	printTimer = displayTime;
	printTimerPreviousMillis = millis();
}

void led5x6PrintString(uint8_t len)
{

    displayModeBackup = I2C_CMD_DISP_STR;
	printFlag = true;
	printMotion = true;
	ledFlashCommand = false;
	ledFlashStatus = false;
    displayOffsetX = 0;
    displayOffsetY = 0;
    displayModeBackup = 0xff;

	if(displayOrientation == DISPLAY_ROTATE_0 || displayOrientation == DISPLAY_ROTATE_180)
    {
        stringLength = len * 5;
        printTimer = displayTimeBackup / len / 5;
    }
    else if(displayOrientation == DISPLAY_ROTATE_90 || displayOrientation == DISPLAY_ROTATE_270)
    {
        stringLength = len * 6;
        printTimer = displayTimeBackup / len / 6;
    }
    
	stringIndex = 0;
	printTimerPreviousMillis = millis();
}

void led5x6PrintData(uint8_t *buffer, uint16_t displayTime)
{
	uint8_t i, j;
    int8_t offset;

    displayModeBackup = I2C_CMD_DISP_DATA;
    displayTimeBackup = displayTime;
    
    if(displayOrientation == DISPLAY_ROTATE_0 || displayOrientation == DISPLAY_ROTATE_180)
    {
        for(i = 0; i < 5; i ++)
        {
            displayCustomBackup[i] = buffer[i];
            displayBuffBackup[i] = displayCustomBackup[i];
        }
          
        ledOffset(5, 6);
    }
    
	printFlag = true;
	printMotion = false;
	printTimer = displayTime;
	printTimerPreviousMillis = millis();
}

void led5x6PrintNumber(int32_t num, uint16_t displayTime)
{
	int16_t temp = 0;
	uint8_t i = 0, bitNum = 0, data[8] = {0,};
    
    displayModeBackup = I2C_CMD_DISP_NUM;
    displayNumberBackup = num;
    displayTimeBackup = displayTime;
	
	if(num >= 0 && num <= 9)
	{
		led5x6PrintAscii((uint8_t)(num + '0'), displayTime);
	}
	else
	{
		if(num < 0)
		{
			num = num * (-1);
			data[i ++] = '-';
		}
		
		temp = num;
		while(temp)
		{
			temp /= 10;
			bitNum ++;
		}
		
		temp = 1;
		for(uint8_t j = 0; j < (bitNum - 1); j ++)temp *= 10;
		
		while(temp)
		{
			data[i ++] = num / temp + '0';
			num %= temp;
			temp /= 10;
		}
		memcpy(stringBuff, data, i);
		stringBuff[i] = 0;
		if(displayTime == 0)
		{
            if(displayOrientation == DISPLAY_ROTATE_0 || displayOrientation == DISPLAY_ROTATE_180)displayTime = i * 5 * 200;
            else if(displayOrientation == DISPLAY_ROTATE_90 || displayOrientation == DISPLAY_ROTATE_270)displayTime = i * 6 * 200;
			
			displayStringFlagBackup = true;
			
            printAlway = true;
		}
		else
		{
			displayStringFlagBackup = false;
			
            printAlway = false;
		}			
		displayStringLenghBackup = i;
		displayTimeBackup = displayTime;
		led5x6PrintString(i);
	}
}

void led5x6SetOffset(void)
{
    switch(displayModeBackup)
    {
        case I2C_CMD_DISP_BAR:
            led5x6PrintBar(displayDataBackup, displayTimeBackup);
        break;
        
        case I2C_CMD_DISP_PHIZ:
            led5x6PrintIcon(displayDataBackup, displayTimeBackup);
        break;
        
        case I2C_CMD_DISP_DATA:
            led5x6PrintData(displayCustomBackup, displayTimeBackup);
        break;
        
        default:
        break;
    }
}
