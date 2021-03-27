
#include "Wire.h"
#include "Flash.h"
#include <LowPower.h>
#include <WatchDog.h>
#include "Timer3.h"
#include "Timer14.h"



#define GROVE_TX_PIN_NUM        PA2
#define GROVE_RX_PIN_NUM        PA3
#define GROVE_LED_PIN_NUM       PA4

/***************************************************************
 Board defines
 ***************************************************************/
#define MATRIX_DI_PIN_NUM       PA7
#define MATRIX_DCK_PIN_NUM      PA5
#define MATRIX_DI_PIN_BIT       0x80
#define MATRIX_DCK_PIN_BIT      0x20
// when LINE_SELECT_EN_PIN is high, V_drive=3.3v, line selector works,
// my9221 works， screen number selector works
#define LINE_SELECT_EN_PIN      PA1
#define LINE_ADDR_0             PF0
#define LINE_ADDR_1             PF1
#define LINE_ADDR_2             PA0
#define LINE_ADDR_0_1_BIT       0x03
#define LINE_ADDR_2_BIT         0x01
#define LINE_EN_BIT             0x02

/***************************************************************

 ***************************************************************/
#define MATRIX_CMD_MODE_0       0x0000
#define MATRIX_CMD_MODE_1       0x0010  //apdm

/***************************************************************

 ***************************************************************/
 // start at ascii 32(dec), for example, '0' = ASCII[48-32]
const uint64_t ASCII[] = {
     0x0000000000000000,     // space
     0x180018183c3c1800,     // !
     0x00000000286c6c00,     // "
     0x6c6cfe6cfe6c6c00,     // #
     0x103c403804781000,     // $
     0x60660c1830660600,     // %
     0xfc66a6143c663c00,     // &
     0x0000000c18181800,     // '
     0x6030181818306000,     // (
     0x060c1818180c0600,     // )
     0x006c38fe386c0000,     // *
     0x0010107c10100000,     // +
     0x060c0c0c00000000,     // ,
     0x0000003c00000000,     // -
     0x0606000000000000,     // .
     0x00060c1830600000,     // /
     0x1c2222222222221c,     // 0
     0x1c08080808080c08,     // 1
     0x3e0408102020221c,     // 2
     0x1c2220201820221c,     // 3
     0x20203e2224283020,     // 4
     0x1c2220201e02023e,     // 5
     0x1c2222221e02221c,     // 6
     0x040404081020203e,     // 7
     0x1c2222221c22221c,     // 8
     0x1c22203c2222221c,     // 9
     0x0018180018180000,     // :
     0x0c18180018180000,     // ;
     0x6030180c18306000,     // <
     0x00003c003c000000,     // =
     0x060c1830180c0600,     // >
     0x1800183860663c00,     // ?
     0x003c421a3a221c00,     // @
     0x0033333f33331e0c,     // A
//   0x0066667e66663c18,     // A
     0x003f66663e66663f,     // B
     0x003c66030303663c,     // C
     0x001f36666666361f,     // D
     0x007f46161e16467f,     // E
     0x000f06161e16467f,     // F
     0x007c66730303663c,     // G
     0x003333333f333333,     // H
     0x001e0c0c0c0c0c1e,     // I
     0x001e333330303078,     // J
     0x006766361e366667,     // K
     0x007f66460606060f,     // L
     0x0063636b7f7f7763,     // M
     0x006363737b6f6763,     // N
     0x001c36636363361c,     // O
     0x000f06063e66663f,     // P
     0x00381e3b3333331e,     // Q
     0x006766363e66663f,     // R
     0x001e33380e07331e,     // S
     0x001e0c0c0c0c2d3f,     // T
     0x003f333333333333,     // U
     0x000c1e3333333333,     // V
     0x0063777f6b636363,     // W
     0x0063361c1c366363,     // X
     0x001e0c0c1e333333,     // Y
     0x007f664c1831637f,     // Z
     0x7818181818187800,     // [
     0x006030180c060000,     // '\'
     0x1e18181818181e00,     // ]
     0x0000008244281000,     // ^
     0x7e00000000000000,     // _
     0x0000000060303000,     // `
     0x006e333e301e0000,     // a
     0x003b66663e060607,     // b
     0x001e3303331e0000,     // c
     0x006e33333e303038,     // d
     0x001e033f331e0000,     // e
     0x000f06060f06361c,     // f
     0x1f303e33336e0000,     // g
     0x006766666e360607,     // h
     0x001e0c0c0c0e000c,     // i
     0x1e33333030300030,     // j
     0x0067361e36660607,     // k
     0x001e0c0c0c0c0c0e,     // l
     0x00636b7f7f330000,     // m
     0x00333333331f0000,     // n
     0x001e3333331e0000,     // o
     0x0f063e66663b0000,     // p
     0x78303e33336e0000,     // q
     0x000f06666e3b0000,     // r
     0x001f301e033e0000,     // s
     0x00182c0c0c3e0c08,     // t
     0x006e333333330000,     // u
     0x000c1e3333330000,     // v
     0x00367f7f6b630000,     // w
     0x0063361c36630000,     // x
     0x1f303e3333330000,     // y
     0x003f260c193f0000,     // z
     0x7018180c18187000,     // {
     0x0008080808080800,     // |
     0x0e18183018180e00,     // }
     0x000000365c000000      // ~
 };

const uint64_t ICONS[] = {
     0xffffffffffffffff,     // 0  full screen
     0x383838fe7c381000,     // 1  arrow_up
     0x10387cfe38383800,     // 2  arrow_down
     0x10307efe7e301000,     // 3  arrow_right
     0x1018fcfefc181000,     // 4  arrow_left
     0xfefe7c7c38381000,     // 5  triangle_up
     0x1038387c7cfefe00,     // 6  triangle_down
     0x061e7efe7e1e0600,     // 7  triangle_right
     0xc0f0fcfefcf0c000,     // 8  triangel_left
     0x7c92aa82aa827c00,     // 9  smile_face_1
     0x003c420000660000,     // 10 smile_face_2
     0x10387cfefeee4400,     // 11 hearts
     0x10387cfe7c381000,     // 12 diamonds
     0x381054fe54381000,     // 13 clubs
     0x38107cfe7c381000,     // 14 spades
     0x00387c7c7c380000,     // 15 circle1
     0xffc7838383c7ffff,     // 16 circle2
     0x0038444444380000,     // 17 circle3
     0xffc7bbbbbbc7ffff,     // 18 circle4
     0x0c12129ca0c0f000,     // 19 man
     0x38444438107c1000,     // 20 woman
     0x060e0c0808281800,     // 21 musical_note_1
     0x066eecc88898f000,     // 22 musical_note_2
     0x105438ee38541000,     // 23 snow
     0x1038541054381000,     // 24 up_down
     0x6666006666666600,     // 25 double_!
     0x002844fe44280000,     // 26 left_right
     0xfe8282c66c381000,     // 27 house
	 0x000000ffff000000,     // 28 ==
    //  0x002400e7a5bde700      // 28 glasses
 };

const uint64_t BARS[] = {
    0x0000000000000000,
    0x1800000000000000,
    0x3c00000000000000,
    0x7e00000000000000,
    0xff00000000000000,
    0xff18000000000000,
    0xff3c000000000000,
    0xff7e000000000000,
    0xffff000000000000,
    0xffff180000000000,
    0xffff3c0000000000,
    0xffff7e0000000000,
    0xffffff0000000000,
    0xffffff1800000000,
    0xffffff3c00000000,
    0xffffff7e00000000,
    0xffffffff00000000,
    0xffffffff18000000,
    0xffffffff3c000000,
    0xffffffff7e000000,
    0xffffffffff000000,
    0xffffffffff180000,
    0xffffffffff3c0000,
    0xffffffffff7e0000,
    0xffffffffffff0000,
    0xffffffffffff1800,
    0xffffffffffff3c00,
    0xffffffffffff7e00,
    0xffffffffffffff00,
    0xffffffffffffff18,
    0xffffffffffffff3c,
    0xffffffffffffff7e,
    0xffffffffffffffff
 };

const uint64_t WAVE = 0x0007888888887000;

const uint64_t COLORFUL_ICONS[] = {

  0xffff1f1f1f1fffff,   // 0 smile
  0xff1f1f1f1f1f1fff,
  0x1f1f9b1f1f9b1f1f,
  0x1f1f1f1f1f1f1f1f,
  0x1f1f1f1f1f1f1f1f,
  0x1f9b1f1f1f1f9b1f,
  0xff1f9b9b9b9b1fff,
  0xffff1f1f1f1fffff,
 
  0xffff1f1f1f1fffff,   // 1 laugh
  0xff1f1f1f1f1f1fff,
  0x1f9b1f1f1f1f9b1f,
  0x9b1f9b1f1f9b1f9b,
  0x1f1f1f1f1f1f1f1f,
  0x1f9b9b9b9b9b9b1f,
  0xff1f9b9b9b9b1fff,
  0xffff1f1f1f1fffff,
 
  0xffff1f1f1f1fffff,   // 2 sad
  0xff1f1f1f1f1f1fff,
  0x1f1f9b1f1f9b1f1f,
  0x1f1f1f1f1f1f1f1f,
  0x1f1f1f1f1f1f1f1f,
  0x1f1f1f9b9b1f1f1f,
  0xff1f9b1f1f9b1fff,
  0xffff1f1f1f1fffff,
 
  0xffff1f1f1f1fffff,   // 3 mad
  0xff9b1f1f1f1f9bff,  
  0x1f1f9b1f1f9b1f1f,
  0x1f1f1f1f1f1f1f1f,
  0x1f1f1f1f1f1f1f1f,
  0x1f1f1f9b9b1f1f1f,
  0xff1f9b1f1f9b1fff,
  0xffff1f1f1f1fffff,
 
  0xfffffdfdfdfdffff,   // 4 angry
  0xfffdfdfdfdfdfdff,
  0xfd1efdfdfdfd1efd,
  0xfdfd1efdfd1efdfd,
  0xfdfdfdfdfdfdfdfd,
  0xfdfd1e1e1e1efdfd,
  0xfffd1e1e1e1efdff,
  0xfffffdfdfdfdffff,
 
  0xffff1f1f1f1fffff,   // 5 cry
  0xff1f1f1f1f1f1fff,
  0x1ffefe1f1ffefe1f,
  0x1f961f1f1f1f961f,
  0x1f961f1f1f1f961f,
  0x1f1f1ffefe1f1f1f,
  0xff1f1ffefe1f1fff,
  0xffff1f1f1f1fffff,
 
  0xffff1f1f1f1fffff,   // 6 greedy
  0xff1f1f1f1f1f1fff,
  0x1f9b9b1f1f9b9b1f,
  0x1f1f1f1f1f1f1f1f,
  0x1f1f1f1f1f1f1f1f,
  0x1f9b9b9b9b9b9b1f,
  0xff1ff7f71f1f1fff,
  0xfffff7f71f1fffff,
 
  0xffff1f1f1f1fffff,   // 7 cool
  0xff1f1f1f1f1f1fff,
  0x7c7cfe7c7c7c7cfe,
  0x7c7c7c1f1f7c7c7c,
  0x7c7c7c1f1f7c7c7c,
  0x1f9b1f1f1f1f1f1f,
  0xff1f9b9b9b9b1fff,
  0xffff1f1f1f1fffff,

  0xffff1f1f1f1fffff,   // 8 shy
  0xff1f1f1f1f1f1fff,
  0x1f1f9b1f1f9b1f1f,
  0x1f1f1f1f1f1f1f1f,
  0xe7e71f1f1f1fe7e7,
  0xe7e71f1f1f1fe7e7,
  0xff1f1f9b9b1f1fff,
  0xffff1f1f1f1fffff,
 
  0xfefe0d0d0d0d0dff,   // 9 awkward
  0xfe0d0d0d0d0d0d0d,
  0x0dfffe0d0dfffe0d,
  0x0dfefe0d0dfefe0d,
  0x0d0d0d0d0d0d0d0d,
  0x0d0d0dffffff0d0d,
  0x0d0d0dff0dff0d0d,
  0xff0d0d0d0d0d0dff,
 
  0xff2121ffff2121ff,   // 10 heart
  0x21fdfd2121fdfd21,
  0x21fdfdfdfdfdfd21,
  0x21fdfdfdfdfdfd21,
  0x21fdfdfdfdfdfd21,
  0xff21fcfcfdfd21ff,
  0xffff21fcfd21ffff,
  0xffffff2121ffffff,
 
  0xffffffffffffffff,   // 11 small heart
  0xff2121ffff2121ff,
  0x21fdfd2121fdfd21,
  0x21fdfdfdfdfdfd21,
  0xff21fdfdfdfd21ff,
  0xffff21fdfd21ffff,
  0xffffff2121ffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,   // 12 broken heart
  0xfffdfdfffffdfdff,
  0xfdfdfdfdfffffdfe,
  0xfdfdfdfffdfdfdfe,
  0xfdfdfdfdfffdfdfd,
  0xfffdfdfffdfdfdff,
  0xfffffdfdfffdffff,
  0xfffffffdffffffff,
 
  0xffffffff8fffffff,   // 13 water
  0xffffff8f8ffeffff,
  0xffff8f8f8ffefeff,
  0xff8f8f8f8f8ffe8f,
  0xff8f8f8f8f8f8f8f,
  0xff8f8f8f8f8ffe8f,
  0xff8f8f8f8f8f8f8f,
  0xffff8f8f8f8f8fff,
 
  0xfffffffff4ffffff,   // 14 fire
  0xfffffffff4ffffff,
  0xfffff4ff1ef4ffff,
  0xfff41ef426f4f4ff,
  0xf4f41e1e26f41ef4,
  0xf41e26fefe261ef4,
  0xf41e26fefe261ef4,
  0xfff41e26261ef4ff,
 
  0x5656565656565656,   // 15 creeper
  0x5656565656565656,
  0x56ffff5656ffff56,
  0x56ffff5656ffff56,
  0x565656ffff565656,
  0x5656ffffffff5656,
  0x5656ffffffff5656,
  0x5656ff5656ff5656,
 
  0x56565656565656fe,   // 16 mad creeper
  0xfeffff5656ffff56,
  0xfefffc5656fcff56,
  0x56565656565656fe,
  0x5656ffffffff5656,
  0xfe56ffffffff5656,
  0x5656ff5656ff56fe,
  0x56fe565656565656,
 
  0x8d9dffffffffffff,   // 17 sword
  0x7f8d9dffffffffff,
  0xff7f8d9dffffffff,
  0xffff7f8d9dff1919,
  0xffffff7f8d9d19ff,
  0xffffffff7f19ffff,
  0xffffff1919ff19ff,
  0xffffff19ffffff19,
 
  0x1812ffffffffffff,   // 18 wooden sword
  0x1e1812ffffffffff,
  0xff1e1812ffffffff,
  0xffff1e1812ff0404,
  0xffffff1e181204ff,
  0xffffffff1e04ffff,
  0xffffff0404ff04ff,
  0xffffff04ffffff04,
 
  0xcbc5ffffffffffff,   // 19 crystal sword
  0xd3cbc5ffffffffff,
  0xffd3cbc5ffffffff,
  0xffffd3cbc5ff7777,
  0xffffffd3cbc577ff,
  0xffffffffd377ffff,
  0xffffff7777ff77ff,
  0xffffff77ffffff77,
 
  0xffffff1f1fffffff,   // 20 house
  0xffff1f1f1f1fffff,
  0xff1f1f1f1f1f1fff,
  0x1f1f1f1f1f1f1f1f,
  0xffa26f6fa2a2a2ff,
  0xffa26f6fa2a2a2ff,
  0xffa2a2a2a2a2a2ff,
  0xffa2a2a2a2a2a2ff,
 
  0xffff61616161ffff,   // 21 tree
  0xff616161616161ff,
  0x6161616161616161,
  0xff616161616161ff,
  0xffff611f1f61ffff,
  0xffffff1f1fffffff,
  0xffffff1f1fffffff,
  0xff1f1f1f1f1f1fff,
 
  0xfffffffdfdffffff,   // 22 flower
  0xfffffd2121fdffff,
  0xfffd21212121fdff,
  0xfffd21212121fdff,
  0xfffffd2121fdffff,
  0xfffffffdfdffffff,
  0xffffff5858ffffff,
  0xff585858585858ff,
 
  0xffffff93ffffffff,   // 23 umbrella
  0xffff938093ffffff,
  0xff9380808093ffff,
  0x93808080808093ff,
  0xffffff1affffffff,
  0xffffff1affffffff,
  0xffffff1affff1aff,
  0xffffff1a1717ffff,
 
  0xffffffffffffffff,   // 24 rain
  0xfffffffefeffffff,
  0xfffffefefefeffff,
  0xfffffefefefeffff,
  0xfffefefefefefefe,
  0xff7effff7effff7e,
  0x7effff7effff7eff,
  0xffffffffffffffff,
 
  0xffa5a5a5a5a5a5ff,   // 25 monster
  0xa5a5a5a5a5a5a5a5,
  0xa5fda5a5fda5a5a5,
  0xa5a5a5a5a5a5a5a5,
  0xa5a5fefefffea5a5,
  0xa5ffffffffffa5a5,
  0xa5fefefffefea5a5,
  0xa5a5a5a5a5a5a5ff,
 
  0xfffffefefffefeff,   // 26 crab
  0xfffffdfefffdfeff,
  0xffffff19ffff19ff,
  0xffffff19ffff19ff,
  0x1919ff1919191919,
  0xff191919ffff1919,
  0x1919ff1919191919,
  0xffffff19ff19ff19,
 
  0xffffffffffffffff,   // 27 duck
  0xffffffff2d2d2dff,
  0xffffffff2dff2dff,
  0xffffffff2d2d1919,
  0x2dff2d2d2d2dffff,
  0x2d2d2d2d2d2d2dff,
  0x2d2d2d2d2d2d2dff,
  0xff2d2d2d2d2dffff,
 
  0xffc3c3ffffc3c3ff,   // 28 rabbit
  0xffc3c3c3c3c3c3ff,
  0xc3c3dbc3c3dbc3c3,
  0xc3c3c3c3c3c3c3c3,
  0xc3c3c3c3c3c3c3c3,
  0xc3c3fffefeffc3c3,
  0xc3c3c3ffffc3c3c3,
  0xffc3c3c3c3c3c3ff,
 
  0xffffff1bffffff1b,   // 29 cat
  0xffffff1b1b1b1b1b,
  0xff1bff1bff1bff1b,
  0x1bffff1b1b031b1b,
  0x24ff1d1d1d1b1bff,
  0x1bff242424ffffff,
  0x241b1d1d1b1bffff,
  0xff1b1b1bff1b1bff,

  0xffffff8080ffffff,   // 30 up
  0xffff80808080ffff,
  0xff808080808080ff,
  0xffffff8080ffffff,
  0xffffff8080ffffff,
  0xffffff8080ffffff,
  0xffffff8080ffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,   // 31 down
  0xffffff8080ffffff,
  0xffffff8080ffffff,
  0xffffff8080ffffff,
  0xffffff8080ffffff,
  0xff808080808080ff,
  0xffff80808080ffff,
  0xffffff8080ffffff,
 
  0xffffffffffffffff,   // 32 left
  0xffffffffff80ffff,
  0xffffffffff8080ff,
  0xff80808080808080,
  0xff80808080808080,
  0xffffffffff8080ff,
  0xffffffffff80ffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,   // 33 right
  0xffff80ffffffffff,
  0xff8080ffffffffff,
  0x80808080808080ff,
  0x80808080808080ff,
  0xff8080ffffffffff,
  0xffff80ffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,   // 34 smile3 
  0xff8080ffff8080ff,
  0xff8080ffff8080ff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xff80ffffffff80ff,
  0xffff80808080ffff,
  0xffffffffffffffff

};

const uint64_t COLORFUL_FRAMES[] = {
 
  0xffffffffffffff00,           // 0 顺时针/大
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xfffffffffffffd80,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xfffffffffffd8080,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xfffffffffd808080,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xfffffffd80808080,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xfffffd8080808080,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xfffd808080808080,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xfd80808080808080,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0x8080808080808080,
  0xfdffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0xfdffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0xfdffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0xfdffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0xfdffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0xfdffffffffffffff,
  0xffffffffffffffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0xfdffffffffffffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80fdffffffffffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x8080fdffffffffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x808080fdffffffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80808080fdffffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x8080808080fdffff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x808080808080fdff,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80808080808080fd,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80fffffffffffffd,
  0x8080808080808080,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80fffffffffffffd,
  0x80ffffffffffff80,
  0x8080808080808080,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80fffffffffffffd,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x8080808080808080,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80ffffffffffffff,
  0x80fffffffffffffd,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x8080808080808080,
 
  0x8080808080808080,
  0x80ffffffffffffff,
  0x80fffffffffffffd,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x8080808080808080,
 
  0x8080808080808080,
  0x80fffffffffffffd,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x8080808080808080,
 
  0x80808080808080fd,           //28
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x80ffffffffffff80,
  0x8080808080808080,

  0xffffffffffffffff,           //29 顺时针/小
  0xffffffffffffffff,
  0xfffffffffffdffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xfffffffffd80ffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xfffffffd8080ffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xfffffd808080ffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffff80808080ffff,
  0xfffffdffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffff80808080ffff,
  0xffff80ffffffffff,
  0xfffffdffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffff80808080ffff,
  0xffff80ffffffffff,
  0xffff80ffffffffff,
  0xfffffdffffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffff80808080ffff,
  0xffff80ffffffffff,
  0xffff80ffffffffff,
  0xffff80fdffffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffff80808080ffff,
  0xffff80ffffffffff,
  0xffff80ffffffffff,
  0xffff8080fdffffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffff80808080ffff,
  0xffff80ffffffffff,
  0xffff80ffffffffff,
  0xffff808080fdffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffff80808080ffff,
  0xffff80ffffffffff,
  0xffff80fffffdffff,
  0xffff80808080ffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,
  0xffffffffffffffff,
  0xffff80808080ffff,
  0xffff80fffffdffff,
  0xffff80ffff80ffff,
  0xffff80808080ffff,
  0xffffffffffffffff,
  0xffffffffffffffff,
 
  0xffffffffffffffff,           //41
  0xffffffffffffffff,
  0xffff808080fdffff,
  0xffff80ffff80ffff,
  0xffff80ffff80ffff,
  0xffff80808080ffff,
  0xffffffffffffffff,
  0xffffffffffffffff,

  0xffff29292929ffff,           // 42 walking
  0xff291c1c1c1c29ff,
  0xff1cff1cff1c29ff,
  0xff1c1c1c1c1c29ff,
  0xffaffdaffdafafff,
  0xffafaffdafaf21ff,
  0xffafafafafaf21ff,
  0xffffff1c1cffffff,
 
  0xffff29292929ffff,
  0xff291c1c1c1c29ff,
  0xff1cff1cff1c29ff,
  0xff1c1c1c1c1c29ff,
  0xffaffdaffdafafff,
  0xffafaffdaf21afff,
  0xffafafafaf21afff,
  0xffff1cff1cffffff,

  0xff2121ffff2121ff,           // 44 heart broken
  0x21fdff2121fdff21,
  0x21ffffffffffff21,
  0x21ffffffffffff21,
  0x21ffffffffffff21,
  0xff21ffffffff21ff,
  0xffff21ffff21ffff,
  0xffffff2121ffffff,
 
  0xff2121ffff2121ff,
  0x21ffff2121fdfd21,
  0x21fdfffdfdfdff21,
  0x21ffffffffffff21,
  0x21ffffffffffff21,
  0xff21ffffffff21ff,
  0xffff21ffff21ffff,
  0xffffff2121ffffff,
 
  0xff2121ffff2121ff,
  0x21fdfd2121fdff21,
  0x21fffffdfffdfd21,
  0x21fdfffdfdfdff21,
  0x21ffffffffffff21,
  0xff21ffffffff21ff,
  0xffff21ffff21ffff,
  0xffffff2121ffffff,
 
  0xff2121ffff2121ff,
  0x21fdfd2121fdfd21,
  0x21fdfdfdfdfdff21,
  0x21fdfffdfffdfd21,
  0x21fdfffdfdfdff21,
  0xff21ffffffff21ff,
  0xffff21ffff21ffff,
  0xffffff2121ffffff,
 
  0xff2121ffff2121ff,
  0x21fffd2121fffd21,
  0x21fdfdfffdfdfd21,
  0x21fdfdfdfdfdff21,
  0x21fdfffdfffdfd21,
  0xff21fffdfdfd21ff,
  0xffff21ffff21ffff,
  0xffffff2121ffffff,
 
  0xff2121ffff2121ff,
  0x21fdff2121fffd21,
  0x21fdfdfdfffdfd21,
  0x21fdfdfffdfdfd21,
  0x21fdfdfdfdfdfd21,
  0xff21fffdfffd21ff,
  0xffff21fdfd21ffff,
  0xffffff2121ffffff,
 
  0xff2121ffff2121ff,
  0x21fdfd2121fffd21,
  0x21fdfffffdfdfd21,
  0x21fdfdfdfffdfd21,
  0x21fdfdfdfdfdfd21,
  0xff21fdfdfdfd21ff,
  0xffff21fdfd21ffff,
  0xffffff2121ffffff,
 
  0xff2121ffff2121ff,
  0x21fdff2121fffd21,
  0x21fdfdfffffdfd21,
  0x21fdfdfdfdfdfd21,
  0x21fdfdfdfdfdfd21,
  0xff21fdfdfdfd21ff,
  0xffff21fdfd21ffff,
  0xffffff2121ffffff,
 
  0xff2121ffff2121ff,           //52
  0x21fdfd2121fdfd21,
  0x21fdfdfdfdfdfd21,
  0x21fdfdfdfdfdfd21,
  0x21fdfdfdfdfdfd21,
  0xff21fcfcfdfd21ff,
  0xffff21fcfd21ffff,
  0xffffff2121ffffff

};

const uint8_t GAMMA_2_2[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 6
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 12
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 18
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 24
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 30
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 36
    0x00, 0x00, 0x00, 0x00, 0x01, 0x01, // 42
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, // 48
    0x01, 0x01, 0x01, 0x02, 0x02, 0x02, // 54
    0x02, 0x02, 0x02, 0x02, 0x03, 0x03, // 60
    0x03, 0x03, 0x03, 0x03, 0x04, 0x04, // 66
    0x04, 0x04, 0x04, 0x05, 0x05, 0x05, // 72
    0x05, 0x05, 0x06, 0x06, 0x06, 0x06, // 78
    0x07, 0x07, 0x07, 0x08, 0x08, 0x08, // 84
    0x09, 0x09, 0x09, 0x0a, 0x0a, 0x0a, // 90
    0x0b, 0x0b, 0x0b, 0x0c, 0x0c, 0x0d, // 96
    0x0d, 0x0d, 0x0e, 0x0e, 0x0f, 0x0f, // 102
    0x10, 0x10, 0x11, 0x11, 0x12, 0x12, // 108
    0x13, 0x13, 0x14, 0x14, 0x15, 0x16, // 114
    0x16, 0x17, 0x17, 0x18, 0x19, 0x19, // 120
    0x1a, 0x1b, 0x1b, 0x1c, 0x1d, 0x1e, // 126
    0x1e, 0x1f, 0x20, 0x21, 0x21, 0x22, // 132
    0x23, 0x24, 0x25, 0x25, 0x26, 0x27, // 138
    0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, // 144
    0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, // 150
    0x34, 0x35, 0x36, 0x37, 0x38, 0x3a, // 156
    0x3b, 0x3c, 0x3d, 0x3e, 0x40, 0x41, // 162
    0x42, 0x43, 0x45, 0x46, 0x47, 0x49, // 168
    0x4a, 0x4b, 0x4d, 0x4e, 0x50, 0x51, // 174
    0x52, 0x54, 0x55, 0x57, 0x58, 0x5a, // 180
    0x5c, 0x5d, 0x5f, 0x60, 0x62, 0x64, // 186
    0x65, 0x67, 0x69, 0x6b, 0x6c, 0x6e, // 192
    0x70, 0x72, 0x74, 0x76, 0x77, 0x79, // 198
    0x7b, 0x7d, 0x7f, 0x81, 0x83, 0x85, // 204
    0x87, 0x89, 0x8c, 0x8e, 0x90, 0x92, // 210
    0x94, 0x96, 0x99, 0x9b, 0x9d, 0x9f, // 216
    0xa2, 0xa4, 0xa6, 0xa9, 0xab, 0xae, // 222
    0xb0, 0xb3, 0xb5, 0xb8, 0xba, 0xbd, // 228
    0xc0, 0xc2, 0xc5, 0xc8, 0xca, 0xcd, // 234
    0xd0, 0xd3, 0xd5, 0xd8, 0xdb, 0xde, // 240
    0xe1, 0xe4, 0xe7, 0xea, 0xed, 0xf0, // 246
    0xf3, 0xf6, 0xf9, 0xfc              // 252
};
uint8_t receiveBuffer[128] = {0,};
uint8_t display_buf[8][16][3] = {0,};
uint8_t RGBFrame[64] = {0xFF,};
/***************************************************************
 Communication defines
 ***************************************************************/
// #define DEVICE_I2C_BASE_ADDRESS 0x60
// change it from 0x10 to 0x60
#define DEVICE_I2C_ADDRESS          0x65
#define DEVICE_VID                  0x2886
#define DEVICE_PID                  0x8005

#define I2C_DEF_ADDR_FLASH_LOC      0x00
#define I2C_CUR_ADDR_FLASH_LOC      0x01

#define I2C_CMD_GET_DEV_ID                      0x00 // This command gets device ID information
#define I2C_CMD_DISP_BAR                        0x01 // This command displays LED bar
#define I2C_CMD_DISP_EMOJI                      0x02 // This command displays emoji
#define I2C_CMD_DISP_NUM                        0x03 // This command displays number
#define I2C_CMD_DISP_STR                        0x04 // This command displays string
#define I2C_CMD_DISP_CUSTOM                     0x05 // This command displays user-defined pictures
#define I2C_CMD_DISP_OFF                        0x06 // This command cleans the display
#define I2C_CMD_DISP_SET                        0x07 // This command set or clear the display by position
#define I2C_CMD_DISP_FLASH                      0x08 // This command displays pictures which are stored in flash
#define I2C_CMD_DISP_COLOR_BAR                  0x09 // This command displays colorful led bar
#define I2C_CMD_DISP_COLOR_WAVE                 0x0a // This command displays built-in wave animation
#define I2C_CMD_DISP_COLOR_CLOCKWISE            0x0b // This command displays built-in clockwise animation
#define I2C_CMD_DISP_COLOR_ANIMATION            0x0c // This command displays other built-in animation
#define I2C_CMD_DISP_COLOR_BLOCK                0x0d // This command displays an user-defined color
#define I2C_CMD_DISP_SWITCH                     0x0e // This command displays an user-defined color
#define I2C_CMD_STORE_FLASH                     0xa0 // This command stores frames in flash
#define I2C_CMD_DELETE_FLASH                    0xa1 // This command deletes all the frames in flash

#define I2C_CMD_CONTINUE_DATA1		0x81

#define I2C_CMD_LED_ON              0xb0 //
#define I2C_CMD_LED_OFF             0xb1 //
#define I2C_CMD_AUTO_SLEEP_ON       0xb2 //
#define I2C_CMD_AUTO_SLEEP_OFF      0xb3 //

#define I2C_CMD_DISP_ROTATE         0xb4 // This command setting the display orientation
#define I2C_CMD_DISP_OFFSET         0xb5 // This command setting the display offset

#define I2C_CMD_SET_ADDR            0xc0 //
#define I2C_CMD_RST_ADDR            0xc1 //

#define I2C_CMD_DISP_GET            0xd1 //

#define I2C_CMD_TEST_TX_RX_ON       0xe0 //
#define I2C_CMD_TEST_TX_RX_OFF      0xe1 //
#define I2C_CMD_TEST_GET_VER        0xe2 //

#define I2C_CMD_DISP_ICON           0xe5 //

#define I2C_CMD_JUMP_TO_BOOT        0xf0 //
#define I2C_CMD_GET_DEVICE_UID      0xf1
#define I2C_CMD_NULL                0xff //

#define DISPLAY_NOTHING             0
#define DISPLAY_SINGLE_CHARACTER    1
#define DISPLAY_MUTLI_CHARACTERS    2
#define DISPLAY_FLASH_BUFFER        3
#define DISPLAY_COLOR_BAR           4
#define DISPLAY_COLOR_EMOJI         5
#define DISPLAY_COLOR_WAVE_GIF      6
#define DISPLAY_COLOR_CLOCKWISE_GIF 7
#define DISPLAY_COLOR_NORMAL_GIF    8
#define DISPLAY_RAINBOW_CYCLE       9
#define DISPLAY_FIRE                10
#define DISPLAY_COLOR_BLOCK         11
#define DISPLAY_SINGLE_PIXEL        12
#define DISPLAY_SWITCH              13
#define DISPLAY_COLOR_ICON          14

#define ACW                         0
#define CW                          1

#define DISPLAY_ROTATE_0            0
#define DISPLAY_ROTATE_90           1
#define DISPLAY_ROTATE_180          2
#define DISPLAY_ROTATE_270          3
#define DISPLAY_ROTATE_135          4

uint8_t base_address = DEVICE_I2C_ADDRESS;
uint16_t device_i2c_address = base_address;
uint8_t commandReceive = I2C_CMD_NULL;

#ifdef BLE_SUPPORT

#define I2C_CMD_LOW_PRIORITY        0x90

bool displayPriorityFlag = false;

uint8_t Core_mode = 0, ErrorCount=0;
uint32_t StartMillis = 0, PreMillis = 0;

typedef struct
{
    uint8_t Datalen;
    uint8_t type;
    uint8_t Address;
    uint8_t Option;
}packet_header_t;

typedef struct
{
    uint8_t     num[2];
}packet_diplay_num;

typedef struct
{
    uint8_t     timer[2];
}packet_diplay_timer;

typedef struct
{
    uint8_t     Raw_data_type;
    uint8_t     delay[2];
}packet_raw_t;

typedef struct
{
    uint8_t     pid[2];
    uint8_t     chipid;
    uint8_t     Newaddress;
    uint8_t     option[5];
}packet_got_atr;

typedef struct
{
    packet_header_t Header;
    union 
    {
        packet_diplay_timer     timer;
        packet_raw_t            raw;
        packet_diplay_num       num;
        packet_got_atr          atr;
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
    packet_header_t Header;
    uint8_t     data[12];
}packet_raw;

typedef struct
{
    packet_header_t Header;
    uint8_t     Event;
}packet_event;

typedef struct
{
    packet_header_t Header;
    uint8_t     pid[2];
    uint8_t     chipid;
    uint8_t     hwaddress;
    uint8_t     version[3];
    uint8_t     option[2];
}packet_atr;

union
{
    packet_atr      atr;
    packet_event    event;
    packet_raw      raw_data;
    uint8_t         bytes[MAINBOARD_BLE_I2C_DATALEN]; 
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

void requestEvent();
void receiveEvent(int howMany);

/***************************************************************
Basic defines
 ***************************************************************/
LowPower nrgSave;

#define CLICK_CHECK_TIMEOUT 1000
#define AUTO_SLEEP_TIMEOUT  2000

uint32_t clickCheckPreviousMillis = 0;
uint32_t autoSleepPreviousMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

#define LED_FLASH_TIME  250

// bool ledFlashCommand = true;
bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;
uint8_t ledFlashTimes = 0;

bool testFlag = false;
char *versions = "V30";
uint16_t NodeVersion = 0x6203;
/* Update for bug FLASH, increase delay time to 12 us from 10 us in latchData, 
       and add BSRR clear in clearDisplay(void).

*/

uint32_t intStart = 0;
uint32_t intEnd = 0;

// display flags and variable
bool display_forever_flag = true;
bool display_stop_flag = false;             // stop display when setting parameters
bool flash_one_time_flag = true;

uint8_t current_line = 0, ContinueNumber = 0;              
uint8_t scroll_mark = 0;                                   
uint32_t scroll_last_time = 0;              
uint32_t scroll_interval_time = 100;        // default scroll interval time

// displayControlThread 
uint32_t display_first_time = 0;
uint32_t display_total_time = 0;

// display list
uint64_t display_list[32] = {ASCII[1], };  // list of display, default display picture is ASCII[1]
uint64_t display_pixel[2] = {0, };
uint8_t display_list_length = 1, display_list_position = 0;
uint8_t display_list_index = 0;            // the displaying character index
uint8_t display_color = 127;               // the displaying color, default color is cyan
// rotataion
uint8_t display_orientation = DISPLAY_ROTATE_0;
// offset
int display_offset_x = 0;
int display_offset_y = 0;

//flash buf
uint32_t flash_buf_interval_time = 1000; //ms
uint32_t flash_buf_last_time = 0;
uint8_t flash_buf_start_pic_num = 0; //0-4
uint8_t flash_buf_end_pic_num = 4;  //0-4
uint8_t flash_buf_now_pic_num = flash_buf_start_pic_num;  //0-4

uint16_t timer3_period = 50000;         // 20ms-50ms
uint8_t RGBData = 0, RawLineNum = 0;
// new
uint8_t display_type;
uint8_t color_icons_index, clockwise_mode;
uint32_t color_block_value;

// fire
const static uint8_t FIRE_COLORS[16][2] = {
    0,0,
    1,0,
    4,0,
    12,0,
    20,0,
    28,0,
    64,0,
    200,0,
    255,0,
    255,1,
    255,2,
    255,4,
    255,8,
    255,10,
    255,16,
    255,20,
};
const static uint8_t FIRE_MASK[8][8] = {
    14, 14, 14, 9,  9,  14, 14, 14,
    16, 16, 12, 9,  9,  12, 16, 16,
    14, 12, 10, 7,  7,  10, 12, 14,
    14, 12,  6,  5,  5, 6,  12, 14,
    8,  8,  6,  2,  2,  6,  8,  8,
    6,  6,  2,  1,  1,  2,  6,  6,
    6,  2,  1,  0,  0,  1,  2,  6,
    2,  1,  0,  0,  0,  0,  1,  2,
};
const static uint8_t FIRE_RANDOMS[255] = {
    10, 14, 9, 11, 10, 13, 6, 11, 12, 7, 15, 11, 13, 12, 15, 14, 
    10, 13, 9, 6, 6, 13, 8, 13, 8, 10, 14, 6, 11, 9, 13, 6, 
    11, 10, 11, 11, 12, 8, 10, 10, 9, 8, 10, 9, 15, 10, 10, 10, 
    7, 8, 12, 9, 11, 10, 10, 11, 7, 11, 8, 12, 10, 15, 12, 13, 
    9, 12, 9, 12, 11, 9, 13, 6, 14, 13, 8, 15, 11, 15, 9, 13, 
    13, 13, 7, 8, 6, 8, 11, 9, 8, 6, 12, 7, 15, 6, 6, 7, 
    10, 6, 13, 12, 13, 11, 7, 11, 9, 6, 12, 13, 14, 10, 15, 11, 
    8, 8, 13, 10, 11, 11, 9, 8, 10, 15, 10, 10, 14, 14, 7, 11, 
    13, 9, 10, 10, 9, 6, 6, 7, 8, 15, 9, 6, 14, 10, 7, 8, 
    7, 8, 14, 6, 11, 9, 9, 12, 14, 11, 10, 13, 8, 11, 6, 8, 
    8, 14, 14, 9, 7, 10, 14, 15, 6, 8, 10, 14, 6, 15, 14, 10, 
    10, 11, 13, 7, 12, 13, 8, 6, 13, 8, 9, 9, 7, 8, 10, 15, 
    9, 12, 11, 12, 9, 7, 12, 7, 9, 9, 11, 9, 13, 14, 6, 7, 
    10, 9, 6, 6, 8, 10, 15, 10, 14, 15, 10, 9, 15, 14, 14, 12, 
    8, 15, 15, 7, 12, 7, 11, 11, 11, 8, 9, 11, 13, 7, 8, 6, 
    7, 6, 7, 11, 12, 6, 14, 9, 8, 13, 10, 8, 15, 8, 8,
};

static int8_t fire_value[8][8] = {0,};
static int8_t fire_line[8] = {0,};
static uint8_t fire_count = 0;
static uint8_t fire_count2 = 0;

/***************************************************************
 Device initialization
 ***************************************************************/
void setup()
{

    uint8_t default_base_address = Flash.read8(I2C_DEF_ADDR_FLASH_LOC);
    uint8_t current_base_address = Flash.read8(I2C_CUR_ADDR_FLASH_LOC);
    if (default_base_address != DEVICE_I2C_ADDRESS) Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
    if (current_base_address == 0xff) Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
    else base_address = current_base_address;
    
    *((uint8_t *)(Flash.blockdata)+I2C_DEF_ADDR_FLASH_LOC) = default_base_address;
    *((uint8_t *)(Flash.blockdata)+I2C_CUR_ADDR_FLASH_LOC) = current_base_address;

    device_i2c_address = base_address;
    memset(RGBFrame, 0xFF, 64);

    packet_01_data.data.deviceVID = DEVICE_VID;
    packet_01_data.data.devicePID = DEVICE_PID;
    packet_01_data.data.deviceEvent = 0;

    nrgSave.begin(GROVE_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance


    scroll_last_time = millis();
    pinMode(GROVE_LED_PIN_NUM, OUTPUT);
    digitalWrite(GROVE_LED_PIN_NUM, LOW);
    matrixInit();
    Wire.begin(device_i2c_address);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    wwdg.begin();

    // display_type = DISPLAY_COLOR_NORMAL_GIF;
    // flash_buf_start_pic_num = 42;
    // flash_buf_now_pic_num = 42;
    // flash_buf_end_pic_num = 43;
    // flash_buf_interval_time = 300;
    // display_forever_flag = true;
#ifdef BLE_SUPPORT
    StartMillis = millis();
#endif
    
    // display_type = DISPLAY_FIRE;
    // flash_buf_interval_time = 40;

    // display_type = DISPLAY_COLOR_BLOCK;
    // display_forever_flag = true;
    // flash_one_time_flag = true;
    // color_block_value = 0x0000ff;
    // send number to display list

    display_first_time = millis();
}

void loop()
{
    uint32_t CurrentMillis = millis();

    if(ledFlashCommand)
    {
        if(CurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
        {
            ledFlashPreviousMillis = CurrentMillis;
            ledFlashStatus = !ledFlashStatus;
        }
    }

#ifdef BLE_SUPPORT
    if (Core_mode == 0)
    {// ATR
        
        if(CurrentMillis - StartMillis >= (I2C_CMD_SYS_READY_TIME+base_address*2))
        {
#if 1
            Core_mode = CORE_BLE_MODE;
#else
            StartMillis = CurrentMillis;
            commandReceive = I2C_CMD_NOTIFY_ATR;
#endif
        }
    }
    if (Core_mode == CORE_BLE_MODE)
    {
        if (RGBData)
        {
            for(RawLineNum=0;RawLineNum<8;RawLineNum++)
            {
                InstructionOption.raw_data.Header.type      = I2C_CMD_GET_RAW_DATA;
                InstructionOption.raw_data.Header.Address   = device_i2c_address;
                InstructionOption.raw_data.Header.Option    = RawLineNum;
                memcpy(InstructionOption.raw_data.data, &RGBFrame[RawLineNum*8], 8);
                InstructionOption.raw_data.Header.Datalen = 12;
                if(Wire.MasterGPIOTransmission(ptr2, InstructionOption.raw_data.Header.Datalen) == 0)
                {
                    ErrorCount++;
                    delay(5);
                }else
                {
                    RawLineNum++;
                    ErrorCount = 0;
                }
                if (ErrorCount > 10)
                {// I2C bus error, reset I2C bus.
                    Wire.end();
                    Wire.begin(device_i2c_address);
                    Wire.onReceive(receiveEvent);
                    Wire.onRequest(requestEvent);
                    ErrorCount = 0;
                }
            }
            RGBData = 0;
        }
        if(commandReceive == I2C_CMD_DISP_GET)
        {
            // r, g, b
            if ((commandOption.data.commands.num.num[0] < 8) && (commandOption.data.commands.num.num[1] < 8))
            {
                InstructionOption.raw_data.Header.type   = I2C_CMD_DISP_GET;
                InstructionOption.raw_data.Header.Address= device_i2c_address;
                InstructionOption.raw_data.Header.Option = commandOption.data.Header.Option;
                InstructionOption.raw_data.data[0]       = display_buf[commandOption.data.commands.num.num[0]][commandOption.data.commands.num.num[1]][0];
                InstructionOption.raw_data.data[1]       = display_buf[commandOption.data.commands.num.num[0]][commandOption.data.commands.num.num[1]][1];
                InstructionOption.raw_data.data[2]       = display_buf[commandOption.data.commands.num.num[0]][commandOption.data.commands.num.num[1]][2];
                InstructionOption.raw_data.Header.Datalen= 7;
                Wire.MasterGPIOTransmission(ptr2, 7);
            }
            commandReceive = I2C_CMD_NULL;
        }else if (commandReceive == I2C_CMD_NOTIFY_ATR)
        {
            InstructionOption.atr.Header.type   = I2C_CMD_ATR;
            InstructionOption.atr.Header.Address= device_i2c_address;
            InstructionOption.atr.pid[0]        = DEVICE_PID&0xff;
            InstructionOption.atr.pid[1]        = (DEVICE_PID>>8 ) & 0xff;
            InstructionOption.atr.chipid        = 0;
            InstructionOption.atr.hwaddress     = DEVICE_I2C_ADDRESS;
            InstructionOption.atr.version[0]    = versions[0];
            InstructionOption.atr.version[1]    = versions[1];
            InstructionOption.atr.version[2]    = versions[2];
            InstructionOption.atr.option[0]     = NodeVersion&0xFF;
            InstructionOption.atr.option[1]     = (NodeVersion>>8)&0xFF;
            InstructionOption.atr.Header.Datalen = sizeof(packet_atr);
            Wire.MasterGPIOTransmission(ptr2, sizeof(packet_atr));
            commandReceive = I2C_CMD_NULL;
        }
    }
#endif
    // change i2c base address
	if(commandReceive == I2C_CMD_SET_ADDR) 
	{
        commandReceive = I2C_CMD_NULL;
        wwdg.end();
        Flash.write8(I2C_CUR_ADDR_FLASH_LOC, base_address);
        wwdg.begin();
        device_i2c_address = base_address;
		Wire.begin(device_i2c_address);
	}
	else if(commandReceive == I2C_CMD_RST_ADDR) // reset i2c address
	{
        commandReceive = I2C_CMD_NULL;
        wwdg.end();
        base_address = Flash.read8(I2C_DEF_ADDR_FLASH_LOC);
        Flash.write8(I2C_CUR_ADDR_FLASH_LOC, base_address);
        wwdg.begin();
        device_i2c_address = base_address;
		Wire.begin(device_i2c_address);
    }else
    // store 
    if (commandReceive == I2C_CMD_STORE_FLASH) 
    {
        commandReceive = I2C_CMD_NULL;
        wwdg.end();
        Flash.writeAll();
        wwdg.begin();
    }
    else if (commandReceive == I2C_CMD_DELETE_FLASH)
    {
        commandReceive = I2C_CMD_NULL;
        // memset((uint8_t *)(Flash.blockdata+48), 0, 320);
        memset((uint8_t *)(Flash.blockdata+48), 0xff, 320);
        wwdg.end();
        Flash.writeAll();
        wwdg.begin();
    }
    
    if(autoSleepFlag)
    {
        uint32_t autoSleepCurrentMillis = millis();
        if((autoSleepCurrentMillis - autoSleepPreviousMillis) > AUTO_SLEEP_TIMEOUT)
        {
            autoSleepPreviousMillis = autoSleepCurrentMillis;
            
            display_list_index = 0;
            Timer14.stop();
            Timer3.stop();
            
            digitalWrite(LINE_SELECT_EN_PIN, LOW);
            digitalWrite(LINE_ADDR_0, LOW);
            digitalWrite(LINE_ADDR_1, LOW);
            digitalWrite(LINE_ADDR_2, LOW);
            digitalWrite(LINE_SELECT_EN_PIN, LOW);

            wwdg.end();
            Wire.end();
            pinMode(PA9, INPUT_PULLUP);
            pinMode(PA10, INPUT_PULLUP);

            nrgSave.standby();

            Wire.begin(device_i2c_address);
            Wire.onReceive(receiveEvent);
            Wire.onRequest(requestEvent);
            
            wwdg.begin();
            matrixInit();
        }
    }

    if(testFlag)
    {
        wwdg.end();
        pinMode(GROVE_TX_PIN_NUM, OUTPUT);
        pinMode(GROVE_RX_PIN_NUM, OUTPUT);

        while(1)
        {
            digitalWrite(GROVE_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_RX_PIN_NUM, HIGH);
            delay(1);
            digitalWrite(GROVE_TX_PIN_NUM, LOW);
            delay(1);

            digitalWrite(GROVE_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_RX_PIN_NUM, LOW);
            delay(1);
            digitalWrite(GROVE_TX_PIN_NUM, LOW);
            delay(1);

            if(testFlag == false)break;
        }

        wwdg.begin();
        attachInterrupt(GROVE_RX_PIN_NUM, dummy, CHANGE, INPUT_PULLUP);
    }

    wwdg.reset();
}

void dummy(void)
{
    autoSleepPreviousMillis = millis();

    if(digitalRead(GROVE_RX_PIN_NUM) == LOW)intStart = millis();
    else
    {
        intEnd = millis();
        if((intEnd - intStart) > 20)delay(500);
        else intStart = intEnd;
    }
}

void receiveEvent(int howMany)
{
    uint8_t i = 0, j = 0;
    // autoSleepPreviousMillis = millis();

    while(Wire.available())
    {
        receiveBuffer[i ++] = Wire.read();
        if(i > 120)i = 0;
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
            {   // High priority task is working.
                commandReceive = I2C_CMD_NULL;
                return;
            }
            commandReceive &= 0x0F;
        }else if (commandReceive < 0x10)
        {
            displayPriorityFlag = true; // High priority
        }else
            displayPriorityFlag = false;
        
        for(j=0;j<(i-sizeof(packet_header_t));j++)
        {
            receiveBuffer[j+1] = receiveBuffer[j+sizeof(packet_header_t)];
        }
        i -= (sizeof(packet_header_t)-1);
    }else
    {
        Core_mode = CORE_ATMEL_MODE;
#endif  
    commandReceive = receiveBuffer[0];
#ifdef BLE_SUPPORT
    }
#endif
    if (ledFlashTimes <= (LED_FLASH_COUNT+10))
    {// Is in splash.
#if 0
        display_stop_flag = true;
        memset(display_buf, 0, 384);
        display_stop_flag = false;
        displayPriorityFlag = false;
        ledFlashTimes = (LED_FLASH_COUNT+20);
        display_total_time = 0;
        display_list[0] = 0;
        display_list[1] = 0;
        display_type = DISPLAY_NOTHING;
        flash_one_time_flag = false;
        display_first_time = 0;
#else
        ledFlashTimes = (LED_FLASH_COUNT+20);
        ledFlashCommand = false;
        display_type = DISPLAY_NOTHING;
#endif
    }
    switch(commandReceive)
    {
#ifdef BLE_SUPPORT
        case I2C_CMD_GET_RAW_DATA:
        // Raw data request, 
            if (commandOption.data.commands.raw.Raw_data_type > 0)
            {
                RGBData = 1;
                RawLineNum = 0;
            }else
                RGBData = 0;
            commandReceive = I2C_CMD_NULL;

            break;
#endif  
        case I2C_CMD_LED_ON:
            ledFlashCommand = true;
            displayPriorityFlag = false;
        break;

        case I2C_CMD_LED_OFF:
            ledFlashCommand = false;
            ledFlashStatus = false;
            displayPriorityFlag = false;
        break;
        // display bar mode
        case I2C_CMD_DISP_BAR:
            display_type = DISPLAY_SINGLE_CHARACTER;
            // send bar number
            if (receiveBuffer[1] > 32)receiveBuffer[1] = 32;
            display_list[0] = BARS[receiveBuffer[1]];
            // set color
            display_color = receiveBuffer[5];
            // set time
            if (receiveBuffer[4]) display_forever_flag = true;
            else display_forever_flag = false;
            flash_one_time_flag = true;
            display_first_time = millis();
            display_total_time = receiveBuffer[2] + receiveBuffer[3] * 256;
        break;
        // display emoji mode
        case I2C_CMD_DISP_EMOJI:
            display_type = DISPLAY_COLOR_EMOJI;
            // static flash one time
            flash_one_time_flag = true;
            color_icons_index = receiveBuffer[1];
            // set time
            if (receiveBuffer[4]) display_forever_flag = true;
            else display_forever_flag = false;
            display_first_time = millis();
            display_total_time = receiveBuffer[2] + receiveBuffer[3] * 256;
        break;
        // display Icon mode
        case I2C_CMD_DISP_ICON:
            display_type = DISPLAY_COLOR_ICON;
            // static flash one time
            flash_one_time_flag = true;
			if (receiveBuffer[1] > 28)break;
            display_list[0] = ICONS[receiveBuffer[1]];
            // set time
			display_color = receiveBuffer[4];
            if (receiveBuffer[5]) display_forever_flag = true;
            else display_forever_flag = false;
            display_first_time = millis();
            display_total_time = receiveBuffer[2] + receiveBuffer[3] * 256;
        break;
        // display number mode
        case I2C_CMD_DISP_NUM:
            // send number to display list
            numberToStringList((int16_t)(receiveBuffer[1]+receiveBuffer[2]*256));
            // set color
            display_color = receiveBuffer[6];
            // set time
            if (receiveBuffer[5]) display_forever_flag = true;
            else display_forever_flag = false;

            display_total_time = receiveBuffer[3] + receiveBuffer[4] * 256;

            if (display_list_length == 1) {
                display_type = DISPLAY_SINGLE_CHARACTER;
                flash_one_time_flag = true;
            }
            else
            {
                display_list[display_list_length++] = ASCII[0];
                // long digitial string scroll
                scroll_interval_time = display_total_time / (8 * (display_list_length - 1));
                // ensure display_total_time is an integral multiple of scroll_interval_time
                display_total_time =  scroll_interval_time * (8 * (display_list_length - 1));
                display_type = DISPLAY_MUTLI_CHARACTERS;
                display_list_position = display_list_length;
                flash_one_time_flag = false;
                // ensure display from the first digit
                display_list_index = 0;
            }
            display_first_time = millis();
        break;
        // display single character or string
        case I2C_CMD_DISP_STR:
            display_list_length = receiveBuffer[4];
            if (display_list_length > 31)display_list_length = 31;
            else if (display_list_length == 0)
            {
                commandReceive = I2C_CMD_NULL;
                break;
            }
            // set color
            display_color = receiveBuffer[5];
            // set time
            if (receiveBuffer[1]) display_forever_flag = true;
            else display_forever_flag = false;

            // display_first_time = millis();
            display_total_time = receiveBuffer[2] + receiveBuffer[3] * 256;

            // get len
            if (display_list_length > 1)
            {
                for (j=0;j<(i-6);j++)
                {
                    display_list[j] = ASCII[receiveBuffer[j+6]-32]; //data starts from receiveBuffer[6]
                }
                display_list_position = j;
                scroll_interval_time = display_total_time / (8 * display_list_length);
                // ensure display_total_time is an integral multiple of scroll_interval_time
                display_total_time =  scroll_interval_time * (8 * display_list_length);
                display_type = DISPLAY_MUTLI_CHARACTERS;
                flash_one_time_flag = false;
                // ensure display from the first digit
                display_list_index = 0;
                if (display_list_position < display_list_length)
                {
                    ContinueNumber = 0;
                }else
                {// String is finished.
                    display_list[display_list_position] = ASCII[0];
                    display_list_length++;
                    display_list_position = display_list_length;
                }
                display_first_time = millis();
            }
            else
            {
                // static display
                display_list[0] = ASCII[receiveBuffer[6]-32]; //data starts from receiveBuffer[6]
                // display_scroll_flag = false;
                display_type = DISPLAY_SINGLE_CHARACTER;
                flash_one_time_flag = true;
                display_first_time = millis();
            }
            
        break;
        // display custom picture's continue packets.
        case I2C_CMD_CONTINUE_DATA:
		case I2C_CMD_CONTINUE_DATA1:
            // display_flash_buf_flag = true;
			if (Core_mode == CORE_BLE_MODE)
			{// Check packet number in BLE mode only.
				if (commandOption.data.Header.Option == ContinueNumber)
				{// Same packet.
					break;
				}
			}
            if (display_type == DISPLAY_NOTHING)break;
            ContinueNumber ++;
            for (j=0;j<(i-1);j++)
            {
                if(display_type == DISPLAY_FLASH_BUFFER)
                {
                    *((uint8_t *)(Flash.blockdata+48+16*flash_buf_now_pic_num)+display_list_position+j) = receiveBuffer[1+j];
                }else if (display_type == DISPLAY_MUTLI_CHARACTERS)
                { /*(display_type == DISPLAY_MUTLI_CHARACTERS)*/
                    display_list[display_list_position+j] = ASCII[receiveBuffer[j+1]-32];
                    if ((display_list_position+j) >= display_list_length)break;
                }
            }
            display_list_position += j;
            if (display_list_position >= display_list_length)
            {
                display_first_time = millis();
                ContinueNumber = 0;
                display_list[display_list_position] = ASCII[0];
                display_list_length++;
                display_list_position = display_list_length;
            }

        break;
        // display custom picture
        case I2C_CMD_DISP_CUSTOM:
            // display_flash_buf_flag = true;
            display_type = DISPLAY_FLASH_BUFFER;
            flash_buf_start_pic_num = 0;
            flash_buf_end_pic_num = receiveBuffer[4] - 1;
            if (flash_buf_start_pic_num == flash_buf_end_pic_num) flash_one_time_flag = true;
            flash_buf_interval_time = display_total_time / receiveBuffer[4];
            display_total_time = flash_buf_interval_time * receiveBuffer[4];
            flash_buf_now_pic_num = receiveBuffer[5];
			if (flash_buf_now_pic_num == 0)
			{// Forever flag and display time in first frame. Was transfered in last time.
				display_forever_flag = receiveBuffer[3];
				display_total_time = receiveBuffer[1] + receiveBuffer[2] * 256;
			}
            display_list_length = 64;
            display_first_time = millis();
            ContinueNumber = 0;
            // data start at receiveBuffer[8]
            for (j=0;j<(i-8);j++)
            {
                *((uint8_t *)(Flash.blockdata+48+16*flash_buf_now_pic_num)+j) = receiveBuffer[8+j];
            }
            display_list_position = j;
        break;

        case I2C_CMD_DISP_FLASH:
            display_type = DISPLAY_FLASH_BUFFER;
            display_list_position = display_list_length = 0;
            display_forever_flag = receiveBuffer[3];
            display_first_time = millis();
            display_total_time = receiveBuffer[1] + receiveBuffer[2] * 256;
            flash_buf_start_pic_num = receiveBuffer[4];
            flash_buf_now_pic_num = flash_buf_start_pic_num;
            flash_buf_end_pic_num = receiveBuffer[5];
            if (flash_buf_start_pic_num == flash_buf_end_pic_num) flash_one_time_flag = true;
            flash_buf_interval_time = display_total_time / (flash_buf_end_pic_num + 1 - flash_buf_start_pic_num);
            display_total_time = flash_buf_interval_time * (flash_buf_end_pic_num + 1 - flash_buf_start_pic_num);
            // read flash data to flash_buf data
            Flash.readAll();
        break;


        case I2C_CMD_DISP_COLOR_BAR:
            display_type = DISPLAY_COLOR_BAR;
            if (receiveBuffer[1] > 32)receiveBuffer[1] = 32;
            display_list[0] = BARS[receiveBuffer[1]];
            flash_one_time_flag = true;
            display_first_time = millis();
            display_total_time = receiveBuffer[2] + receiveBuffer[3] * 256;
            display_forever_flag = receiveBuffer[4];
        break;

        case I2C_CMD_DISP_COLOR_WAVE:
            display_type = DISPLAY_COLOR_WAVE_GIF;
            flash_one_time_flag = true;
            display_color = receiveBuffer[1];
            display_first_time = millis();
            display_total_time = receiveBuffer[2] + receiveBuffer[3] * 256;
            scroll_interval_time = display_total_time / 32;  // 4 * 8 = 32 
            // ensure display_total_time is an integral multiple of scroll_interval_time
            display_total_time =  scroll_interval_time * 32;
            display_forever_flag = receiveBuffer[4];
        break;

        case I2C_CMD_DISP_COLOR_CLOCKWISE:
            display_type = DISPLAY_COLOR_CLOCKWISE_GIF;
            clockwise_mode = receiveBuffer[1];
            display_total_time = receiveBuffer[3] + receiveBuffer[4] * 256;
            if (receiveBuffer[2] == 0)  // small 
            {
                flash_buf_start_pic_num = 29;
                flash_buf_end_pic_num = 41;
            }
            else        // big
            {
                flash_buf_start_pic_num = 0;
                flash_buf_end_pic_num = 28;
            }
            flash_buf_now_pic_num = flash_buf_start_pic_num;
            flash_buf_interval_time = display_total_time / (flash_buf_end_pic_num + 1 - flash_buf_start_pic_num);
            display_total_time = flash_buf_interval_time * (flash_buf_end_pic_num + 1 - flash_buf_start_pic_num);
            display_first_time = millis();
            display_forever_flag = receiveBuffer[5];
        break;

        case I2C_CMD_DISP_COLOR_ANIMATION:
            display_type = DISPLAY_COLOR_NORMAL_GIF;
            flash_buf_start_pic_num = receiveBuffer[1];
            flash_buf_now_pic_num = flash_buf_start_pic_num;
            flash_buf_end_pic_num = receiveBuffer[2];
            display_total_time = receiveBuffer[3] + receiveBuffer[4] * 256;
            flash_buf_interval_time = display_total_time / (flash_buf_end_pic_num + 1 - flash_buf_start_pic_num);
            display_total_time = flash_buf_interval_time * (flash_buf_end_pic_num + 1 - flash_buf_start_pic_num);
            display_first_time = millis();
            display_forever_flag = receiveBuffer[5];
            if ((flash_buf_start_pic_num == 255) && (flash_buf_end_pic_num = 255)) display_type = DISPLAY_RAINBOW_CYCLE;
            else if ((flash_buf_start_pic_num == 254) && (flash_buf_end_pic_num = 254)) display_type = DISPLAY_FIRE;
            else if ((flash_buf_start_pic_num == 42) && (flash_buf_end_pic_num = 43)) flash_buf_interval_time = flash_buf_interval_time / 4;
        break;

        case I2C_CMD_DISP_COLOR_BLOCK:
            display_type = DISPLAY_COLOR_BLOCK;
            // r, g, b
            color_block_value = (uint32_t)(receiveBuffer[1] << 16) + (uint32_t)(receiveBuffer[2] << 8) + (uint32_t)receiveBuffer[3];
            flash_one_time_flag = true;
            display_total_time = receiveBuffer[4] + receiveBuffer[5] * 256;
            display_forever_flag = receiveBuffer[6];
            display_first_time = millis();
        break;

        case I2C_CMD_DISP_OFF:
            display_type = DISPLAY_NOTHING;
            display_forever_flag = false;
            display_total_time = 0;
            displayPriorityFlag = false;
        break;
        
        case I2C_CMD_DISP_SET:
            // r, g, b
            display_color = receiveBuffer[3];
            display_first_time = 1;
            display_type = DISPLAY_SINGLE_PIXEL;
            commandReceive = I2C_CMD_NULL;
        break;  
        case I2C_CMD_DISP_SWITCH:
            // r, g, b
            display_color = receiveBuffer[3];
            display_first_time = 1;
            display_type = DISPLAY_SWITCH;
            commandReceive = I2C_CMD_NULL;
        break;
        // set rotate 
        case I2C_CMD_DISP_ROTATE:
            display_orientation = receiveBuffer[1];
            flash_one_time_flag = true;
        break;
        // set offset
        case I2C_CMD_DISP_OFFSET:
            display_offset_x = (int)receiveBuffer[1] - 8;
            display_offset_y = (int)receiveBuffer[2] - 8;
            flash_one_time_flag = true;
        break;

        case I2C_CMD_AUTO_SLEEP_ON:
            autoSleepFlag = true;
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_AUTO_SLEEP_OFF:
            autoSleepFlag = false;
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_SET_ADDR:
			base_address = receiveBuffer[1];
										  
		break;
        case I2C_CMD_RST_ADDR:
			base_address = DEVICE_I2C_ADDRESS;
            commandReceive = I2C_CMD_NULL;
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
            Wire.write((char *)(Flash.getChipUniqueID()), 12);
            commandReceive = I2C_CMD_NULL;
        break;

        default:
        break;
    }
}

/***************************************************************
 Device driver
 ***************************************************************/
void matrixInit()
{
    pinMode(MATRIX_DCK_PIN_NUM, OUTPUT);
    pinMode(MATRIX_DI_PIN_NUM, OUTPUT);
    pinMode(LINE_ADDR_0, OUTPUT);
    pinMode(LINE_ADDR_1, OUTPUT);
    pinMode(LINE_ADDR_2, OUTPUT);
    pinMode(LINE_SELECT_EN_PIN, OUTPUT);
    digitalWrite(MATRIX_DCK_PIN_NUM, LOW);
    digitalWrite(MATRIX_DI_PIN_NUM, LOW);
    digitalWrite(LINE_ADDR_0, LOW);
    digitalWrite(LINE_ADDR_1, LOW);
    digitalWrite(LINE_ADDR_2, LOW);
    digitalWrite(LINE_SELECT_EN_PIN, HIGH);
    clearDisplay();

    Timer3.init(timer3_period); // 50ms timer
    Timer3.attachInterrupt(displayControlThread);
    // 40fps => 40*8 hz => 3.3ms oneline
    // 1ms * 8lines => 8ms => 125Hz
    // displayOneLine() takes about 0.4ms
    Timer14.init(1000);
    Timer14.attachInterrupt(timerIsr);

    // get a fire line
    for (uint8_t i=0;i<8;i++)
    {
        if ((i == 3)||(i==4))
        {
            while(FIRE_RANDOMS[fire_count2]<13)
            {
                fire_count2++;
            }
        }
        fire_line[i] = FIRE_RANDOMS[fire_count2];
        fire_count2++; 
    }
}


 void timerIsr()
 {
   //An Ideal ISR has to be short and not make any function calls
   //But, in this case only data exchange happens.
   // blink the led PA1 
//    GPIOA->BRR = 0x02;
   displayOneLine();
//    GPIOA->BSRR = 0x02;
 }


// Routine to send 16bit data to MY9221 driver chips
// send MSB first
void send16BitData(uint16_t data)
{
    for (uint8_t i = 0; i < 16; i++)
    {
      GPIOA->BRR = MATRIX_DI_PIN_BIT;
      GPIOA->BSRR = ((1 && (data & 0x8000)) << 7); //if (data & 0x8000){GPIOA->BSRR = DI_PIN_BIT;}
      GPIOA->ODR ^= MATRIX_DCK_PIN_BIT;
      data <<= 1;
    }
}

// clear data in MY9221
void clearDisplay(void)
{
    send16BitData(MATRIX_CMD_MODE_1);
    // set data pin MATRIX_DI_PIN_NUM to low
    // BRR to set low, BSRR to set high
	GPIOA->BRR = MATRIX_DI_PIN_BIT;
	GPIOA->BSRR = 0;
    for (uint8_t i=0;i<192;i++)
    {
        GPIOA->ODR ^= MATRIX_DCK_PIN_BIT;
    }

    send16BitData(MATRIX_CMD_MODE_1);
    // set data pin MATRIX_DI_PIN_NUM to low
    // BRR to set low, BSRR to set high
	GPIOA->BRR = MATRIX_DI_PIN_BIT;
	GPIOA->BSRR = 0;
    for (uint8_t i=0;i<192;i++)
    {
        GPIOA->ODR ^= MATRIX_DCK_PIN_BIT;
    }

    latchData();
}

// line 0-7
void switchToLine(uint8_t line)
{
    // set PF0 PF1 PA0 to low
    GPIOF->BRR = LINE_ADDR_0_1_BIT;
    GPIOA->BRR = LINE_ADDR_2_BIT;
    // set line
    GPIOF->BSRR = (LINE_ADDR_0_1_BIT & line);
    GPIOA->BSRR = (LINE_ADDR_2_BIT & (line>>2));
    // enable the line selector PA4
    GPIOA->BSRR = LINE_EN_BIT;
}

void latchData(void)
{
    //why not 220us ? need to test
    delayMicroseconds(12);
    // send 4 DI pulses
    GPIOA->BRR = MATRIX_DI_PIN_BIT;
    for (uint8_t i=0;i<8;i++)
    {
        GPIOA->ODR ^= MATRIX_DI_PIN_BIT;
    }
}

// clearDisplay() takes 0.13ms
// send16BitData() totally takes 0.26ms
void displayOneLine(void)
{
    uint8_t scroll_mark_copy = 0;
    if ((display_type == DISPLAY_MUTLI_CHARACTERS) || (display_type == DISPLAY_COLOR_WAVE_GIF)) scroll_mark_copy = scroll_mark;
    uint8_t line = (current_line & 0x07);

    if (display_stop_flag == false)
    {
        clearDisplay();
        // clearDisplay();

        send16BitData(MATRIX_CMD_MODE_1);
        // blue
        send16BitData(display_buf[line][7+scroll_mark_copy][2]);
        send16BitData(display_buf[line][6+scroll_mark_copy][2]);
        send16BitData(display_buf[line][5+scroll_mark_copy][2]);
        send16BitData(display_buf[line][4+scroll_mark_copy][2]);
        send16BitData(display_buf[line][3+scroll_mark_copy][2]);
        send16BitData(display_buf[line][2+scroll_mark_copy][2]);
        send16BitData(display_buf[line][1+scroll_mark_copy][2]);
        send16BitData(display_buf[line][0+scroll_mark_copy][2]);

        // green
        send16BitData(display_buf[line][7+scroll_mark_copy][1]);
        send16BitData(display_buf[line][6+scroll_mark_copy][1]);
        send16BitData(display_buf[line][5+scroll_mark_copy][1]);
        send16BitData(display_buf[line][4+scroll_mark_copy][1]);

        send16BitData(MATRIX_CMD_MODE_1);

        send16BitData(display_buf[line][3+scroll_mark_copy][1]);
        send16BitData(display_buf[line][2+scroll_mark_copy][1]);
        send16BitData(display_buf[line][1+scroll_mark_copy][1]);
        send16BitData(display_buf[line][0+scroll_mark_copy][1]);

        // red
        send16BitData(display_buf[line][7+scroll_mark_copy][0]);
        send16BitData(display_buf[line][6+scroll_mark_copy][0]);
        send16BitData(display_buf[line][5+scroll_mark_copy][0]);
        send16BitData(display_buf[line][4+scroll_mark_copy][0]);
        send16BitData(display_buf[line][3+scroll_mark_copy][0]);
        send16BitData(display_buf[line][2+scroll_mark_copy][0]);
        send16BitData(display_buf[line][1+scroll_mark_copy][0]);
        send16BitData(display_buf[line][0+scroll_mark_copy][0]);
        //flash from button line
        switchToLine(7-line);
        current_line++;

        latchData();


    }
}

/***************************************************************
 name:      writeDisplayBuf(1)
 function:  write a 8byte character to displaybuf after rotation
            and offset.
 para:      @data: 8byte character data, such as ASCII[0]
            @color_num: 0-255, 254 is white and 255 is black
 global:    
            @display_buf, @temp_rotation_buf
 ***************************************************************/
void writeDisplayBuf(uint64_t data, uint8_t color_num)
{
    // get character color
    uint32_t rgb = wheel(color_num);
    uint8_t red   = (uint8_t)(rgb >> 16);
    uint8_t green = (uint8_t)(rgb >> 8);
    uint8_t blue  = (uint8_t)(rgb);
    // clear the whole 384bytes buff

    for (uint8_t i=0;i<8;i++)
    {
        for (uint8_t j=0;j<8;j++)
        {
            if (data & 0x1)
            {
                RGBFrame[i*8+j] = color_num;
                display_buf[i][j][0] = red;
                display_buf[i][j][1] = green;
                display_buf[i][j][2] = blue;
            }
            // to read the next bit
            data >>= 1;
        }
    }
}


/***************************************************************
 name:      writeSpareBuf
 function:  write a 8byte character to spare display buf[8]-[15]
            after rotation, no offset.
            Only for scrolling.
 para:      @data: 8byte character data, such as ASCII[0]
            @color_num: 0-255, 254 is white and 255 is black
 global:    @display_buf
 ***************************************************************/
void writeSpareBuf(uint64_t data, uint8_t color_num)
{
    // spare buff doesn't need memset
    uint32_t rgb = wheel(color_num);
    uint8_t red   = (uint8_t)(rgb >> 16);
    uint8_t green = (uint8_t)(rgb >> 8);
    uint8_t blue  = (uint8_t)(rgb);

    for (uint8_t i=0;i<8;i++)
    {
        for (uint8_t j=8;j<16;j++)
        {
            if (data & 0x1)
            {
                RGBFrame[i*8+j] = color_num;
                display_buf[i][j][2] = blue;
                display_buf[i][j][1] = green;
                display_buf[i][j][0] = red;
            }
            else
            {
                RGBFrame[i*8+j] = 0xFF;
                display_buf[i][j][2] = 0;
                display_buf[i][j][1] = 0;
                display_buf[i][j][0] = 0;
            }
            // to read the next bit
            data >>= 1;
        }
    }
}

/***************************************************************
 name:      writeDisplayBuf(2)
 function:  write a 64byte picture to displaybuf, no rotation
            and offset.
 para:      @buf: uint32_t pointer of the picture buf(Flash.blockdata)
 global:    @display_buf
 ***************************************************************/
void writeDisplayBuf(uint32_t *buf)
{
    uint32_t rgb = 0;
    uint8_t red   = 0;
    uint8_t green = 0;
    uint8_t blue  = 0;
    uint8_t count = 0;

    // memset(display_buf, 0, 384);
    memcpy(RGBFrame, buf, 64);

    for (uint8_t i = 0; i < 8; i++)
    {
        for (uint8_t j = 0; j < 8; j++)
        {
            rgb = wheel(*((uint8_t *)buf + count));
            count++;
            red = (uint8_t)(rgb >> 16);
            green = (uint8_t)(rgb >> 8);
            blue = (uint8_t)rgb;
            display_buf[i][j][2] = blue;
            display_buf[i][j][1] = green;
            display_buf[i][j][0] = red;
        }
    }
}

/***************************************************************
 name:      writeDisplayBufMirror
 function:  write a 64byte picture to displaybuf, 并对其进行45度镜像
 para:      @buf: uint32_t pointer of the picture buf(Flash.blockdata)
 global:    @display_buf
 ***************************************************************/
void writeDisplayBufMirror(uint32_t *buf)
{
    uint32_t rgb = 0;
    uint8_t red   = 0;
    uint8_t green = 0;
    uint8_t blue  = 0;
    uint8_t count = 0;

    memset(display_buf, 0, 384);

    for (uint8_t i = 0; i < 8; i++)
    {
        for (uint8_t j=0;j<8;j++)
        {
            rgb = wheel(*((uint8_t *)buf + count));
            count++;
            red = (uint8_t)(rgb >> 16);
            green = (uint8_t)(rgb >> 8);
            blue = (uint8_t)rgb;
            display_buf[j][i][2] = blue;
            display_buf[j][i][1] = green;
            display_buf[j][i][0] = red;
        }
    }
}

/***************************************************************
 name:      writeDisplayBufColorBarMode
 function:  

 para:      @data: 8byte character bar data
 ***************************************************************/
void writeDisplayBufColorBarMode(uint64_t data)
{
    // no rotation and offset
    uint32_t rgb;
    uint8_t red, green, blue;
    const static uint8_t BAR_COLORS[8] = {0x00, 0x05, 0x0c, 0x13, 0x1a, 0x26, 0x26, 0x26};
    // clear the whole 384bytes buff
    memset(display_buf, 0, 384);

    for (uint8_t i=0;i<8;i++)
    {
        rgb = wheel(BAR_COLORS[i]);
        red   = (uint8_t)(rgb >> 16);
        green = (uint8_t)(rgb >> 8);
        blue  = (uint8_t)(rgb); 

        for (uint8_t j=0;j<8;j++)
        {
            if (data & 0x1)
            {
                RGBFrame[i*8+j] = BAR_COLORS[i];
                display_buf[i][j][0] = red;
                display_buf[i][j][1] = green;
                display_buf[i][j][2] = blue;
            }else
            {
                RGBFrame[i*8+j] = 0xFF;
            }
            // to read the next bit
            data >>= 1;
        }
    }
}

/***************************************************************
 name:      adjustDisplayBuf
 function:  

 para:      
 ***************************************************************/
void adjustDisplayBuf(void)
{
    uint8_t i,j,k;
    uint8_t temp, orientation;
    orientation = display_orientation;

    switch(orientation)
    {
        case DISPLAY_ROTATE_90:
            for (i=0;i<4;i++)
            {
                for (j=0;j<4;j++)
                {
                    for (k=0;k<3;k++)
                    {
                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[j][7-i][k];
                        display_buf[j][7-i][k] = temp;

                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[7-i][7-j][k];
                        display_buf[7-i][7-j][k] = temp;

                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[7-j][i][k];
                        display_buf[7-j][i][k] = temp;
                    }
                }
            }
        break;

        case DISPLAY_ROTATE_180:
            for (i=0;i<8;i++)
            {
                for (j=0;j<4;j++)
                {
                    for (k=0;k<3;k++)
                    {
                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[7-i][7-j][k];
                        display_buf[7-i][7-j][k] = temp;
                    }
                }
            }
        break;

        case DISPLAY_ROTATE_270:
            for (i=0;i<4;i++)
            {
                for (j=0;j<4;j++)
                {
                    for (k=0;k<3;k++)
                    {
                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[7-j][i][k];
                        display_buf[7-j][i][k] = temp;

                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[7-i][7-j][k];
                        display_buf[7-i][7-j][k] = temp;

                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[j][7-i][k];
                        display_buf[j][7-i][k] = temp;
                    }
                }
            }
        break;

        case DISPLAY_ROTATE_135:
            for (i=0;i<8;i++)
            {
                for (j=i;j<8;j++)
                {
                    for (k=0;k<3;k++)
                    {
                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[j][i][k];
                        display_buf[j][i][k] = temp;
                    }
                }
            }
        break;

        default:
        break;
    }


    // 右移n格 -8<=n<=8
    int8_t x = display_offset_x;
    int8_t y = display_offset_y;
    int8_t m,n;

    if (x > 0)
    {
        for (m = 7-x; m >= 0; m--)
        {
            for (n = 0; n < 8; n++)
            {
                display_buf[n][m+x][0] = display_buf[n][m][0];
                display_buf[n][m+x][1] = display_buf[n][m][1];
                display_buf[n][m+x][2] = display_buf[n][m][2];
            }
        }
        // x>8
        if (x > 8) x = 8;
        for (m = 0; m < x; m++)
        {
            for (n = 0; n < 8; n++)
            {
                display_buf[n][m][0] = 0;
                display_buf[n][m][1] = 0;
                display_buf[n][m][2] = 0;
            }
        }
    }
    else if (x < 0)
    {
        for (m = (-x); m < 8; m++)
        {
            for (n = 0; n < 8; n++)
            {
                display_buf[n][m+x][0] = display_buf[n][m][0];
                display_buf[n][m+x][1] = display_buf[n][m][1];
                display_buf[n][m+x][2] = display_buf[n][m][2];
            }
        }
        // x < -8
        if (x < -8) x = -8;
        for (m = (8 + x); m < 8; m++)
        {
            for (n = 0; n < 8; n++)
            {
                display_buf[n][m][0] = 0;
                display_buf[n][m][1] = 0;
                display_buf[n][m][2] = 0;
            }
        }
    }
    
    // 向上移动n格 -8<=n<=8
    if (y > 0)
    {
        for (m = y; m < 8; m++)
        {
            for (n = 0; n < 8; n++)
            {
                display_buf[m-y][n][0] = display_buf[m][n][0];
                display_buf[m-y][n][1] = display_buf[m][n][1];
                display_buf[m-y][n][2] = display_buf[m][n][2];
            }
        }
        // y>8
        if (y > 8) y = 8;
        for (m = (8-y); m < 8; m++)
        {
            for (n = 0; n < 8; n++)
            {
                display_buf[m][n][0] = 0;
                display_buf[m][n][1] = 0;
                display_buf[m][n][2] = 0;
            }
        }
    }
    else if (y < 0)
    {
        for (m = (7 + y); m >= 0; m--)
        {
            for (n = 0; n < 8; n++)
            {
                display_buf[m-y][n][0] = display_buf[m][n][0];
                display_buf[m-y][n][1] = display_buf[m][n][1];
                display_buf[m-y][n][2] = display_buf[m][n][2];
            }
        }
        // y < -8
        if (y < -8) y = -8;
        for (m = 0; m < (-y); m++)
        {
            for (n = 0; n < 8; n++)
            {
                display_buf[m][n][0] = 0;
                display_buf[m][n][1] = 0;
                display_buf[m][n][2] = 0;
            }
        }

    }
}

void adjustDisplayBuf2(void)
{
    uint8_t i,j,k;
    uint8_t temp, orientation;
    orientation = display_orientation;

    switch(orientation)
    {
        case DISPLAY_ROTATE_90:
            for (i=0;i<4;i++)
            {
                for (j=0;j<4;j++)
                {
                    for (k=0;k<3;k++)
                    {
                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[j][7-i][k];
                        display_buf[j][7-i][k] = temp;

                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[7-i][7-j][k];
                        display_buf[7-i][7-j][k] = temp;

                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[7-j][i][k];
                        display_buf[7-j][i][k] = temp;

                        temp = display_buf[i][j+8][k];
                        display_buf[i][j+8][k] = display_buf[j][15-i][k];
                        display_buf[j][15-i][k] = temp;

                        temp = display_buf[i][j+8][k];
                        display_buf[i][j+8][k] = display_buf[7-i][15-j][k];
                        display_buf[7-i][15-j][k] = temp;

                        temp = display_buf[i][j+8][k];
                        display_buf[i][j+8][k] = display_buf[7-j][i+8][k];
                        display_buf[7-j][i+8][k] = temp;
                    }
                }
            }
        break;

        case DISPLAY_ROTATE_180:
            for (i=0;i<8;i++)
            {
                for (j=0;j<4;j++)
                {
                    for (k=0;k<3;k++)
                    {
                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[7-i][7-j][k];
                        display_buf[7-i][7-j][k] = temp;

                        temp = display_buf[i][j+8][k];
                        display_buf[i][j+8][k] = display_buf[7-i][15-j][k];
                        display_buf[7-i][15-j][k] = temp;
                    }
                }
            }
        break;

        case DISPLAY_ROTATE_270:
            for (i=0;i<4;i++)
            {
                for (j=0;j<4;j++)
                {
                    for (k=0;k<3;k++)
                    {
                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[7-j][i][k];
                        display_buf[7-j][i][k] = temp;

                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[7-i][7-j][k];
                        display_buf[7-i][7-j][k] = temp;

                        temp = display_buf[i][j][k];
                        display_buf[i][j][k] = display_buf[j][7-i][k];
                        display_buf[j][7-i][k] = temp;

                        temp = display_buf[i][j+8][k];
                        display_buf[i][j+8][k] = display_buf[7-j][i+8][k];
                        display_buf[7-j][i+8][k] = temp;

                        temp = display_buf[i][j+8][k];
                        display_buf[i][j+8][k] = display_buf[7-i][15-j][k];
                        display_buf[7-i][15-j][k] = temp;

                        temp = display_buf[i][j+8][k];
                        display_buf[i][j+8][k] = display_buf[j][7+8-i][k];
                        display_buf[j][7+8-i][k] = temp;
                    }
                }
            }
        break;
    }
}



/***************************************************************
 name:      displayControlThread
 function:
 para:
 global:
 ***************************************************************/
void displayControlThread(void)
{
    if (ledFlashTimes == (LED_FLASH_COUNT+20))
    {// Clear splash content.
        display_stop_flag = true;
        memset(display_buf, 0, 384);
        memset(RGBFrame, 0xFF, 64);
        display_stop_flag = false;
        displayPriorityFlag = false;
        ledFlashTimes++;
    }
    switch(display_type){
        case DISPLAY_NOTHING:  //clear screen
            display_stop_flag = true;
            memset(display_buf, 0, 384);
            memset(RGBFrame, 0xFF, 64);
            display_stop_flag = false;
            displayPriorityFlag = false;
        break;

        case DISPLAY_SINGLE_CHARACTER: //display_list[0]
            //check time out
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {
                if (ledFlashTimes <= (LED_FLASH_COUNT+5))
                {// start animation
                    display_type = DISPLAY_RAINBOW_CYCLE;
                    flash_buf_start_pic_num = 255;
                    flash_buf_now_pic_num   = flash_buf_start_pic_num;
                    flash_buf_end_pic_num   = 255;
                    display_total_time = 2000;
                    flash_buf_interval_time = display_total_time;
                    display_total_time = flash_buf_interval_time;
                    display_first_time = millis();
                    display_forever_flag = 0;
                    ledFlashTimes = LED_FLASH_COUNT+10;
                }else
                {
                    display_type = DISPLAY_NOTHING;
                }
            }
            else
            {
                if (flash_one_time_flag)
                {                
                    flash_one_time_flag = false;
                    display_stop_flag = true;
                    memset(display_buf, 0, 384);
                    writeDisplayBuf(display_list[0], display_color);
                    adjustDisplayBuf();
                    display_stop_flag = false;
                }
            }
        break;

		case DISPLAY_COLOR_ICON: //display_list[0]
            //check time out
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {
                display_type = DISPLAY_NOTHING;
            }
            else
            {
                if (flash_one_time_flag)
                {                
                    flash_one_time_flag = false;
                    display_stop_flag = true;
                    memset(display_buf, 0, 384);
                    writeDisplayBuf(display_list[0], display_color);
                    adjustDisplayBuf();
                    display_stop_flag = false;
                }
            }
        break;
		
        case DISPLAY_SINGLE_PIXEL: //display_list[0]
            if (display_first_time == 0)break;
            if ((commandOption.data.commands.num.num[0] < 8) && (commandOption.data.commands.num.num[1] < 8))
            {
                display_first_time = 0;
                display_pixel[0] = ((uint64_t)1 << (commandOption.data.commands.num.num[0]*8+commandOption.data.commands.num.num[1]));
                display_pixel[1] |= ((uint64_t)1 << (commandOption.data.commands.num.num[0]*8+commandOption.data.commands.num.num[1]));
                {
                    display_stop_flag = true;
                    writeDisplayBuf(display_pixel[0], display_color);
                    adjustDisplayBuf();
                    display_stop_flag = false;
                }
            }
        break;
        
        case DISPLAY_SWITCH: //display_list[0]
            if (display_first_time == 0)break;
            if ((commandOption.data.commands.num.num[0] < 8) && (commandOption.data.commands.num.num[1] < 8))
            {
                display_first_time = 0;
                display_pixel[0] = ((uint64_t)1 << (commandOption.data.commands.num.num[0]*8+commandOption.data.commands.num.num[1]));
                if ((display_buf[commandOption.data.commands.num.num[0]][commandOption.data.commands.num.num[1]][0] == 0) &&
                    (display_buf[commandOption.data.commands.num.num[0]][commandOption.data.commands.num.num[1]][1] == 0) &&
                    (display_buf[commandOption.data.commands.num.num[0]][commandOption.data.commands.num.num[1]][2] == 0) )
                    {// Is in Off status.
                        display_stop_flag = true;
                        writeDisplayBuf(display_pixel[0], display_color);
                        adjustDisplayBuf();
                        display_stop_flag = false;
                    }else
                    {// Turn off.
                        display_stop_flag = true;
                        writeDisplayBuf(display_pixel[0], 255);
                        adjustDisplayBuf();
                        display_stop_flag = false;
                    }
            }
        break;

        case DISPLAY_MUTLI_CHARACTERS: //display_list[]
            if (display_list_position < display_list_length)
            {
                 if ((millis() - display_first_time) >= 2000)
                 {// Lost packet, timeout.
                     display_type = DISPLAY_NOTHING;
                     display_list_position = 0;
                     display_list_length = 0;
                 }
                break;
            }
           // check if it is time out
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {   
                if (ledFlashTimes <= (LED_FLASH_COUNT+5))
                {// start animation
                    display_type = DISPLAY_RAINBOW_CYCLE;
                    flash_buf_start_pic_num = 255;
                    flash_buf_now_pic_num   = flash_buf_start_pic_num;
                    flash_buf_end_pic_num   = 255;
                    display_total_time = 2000;
                    flash_buf_interval_time = display_total_time;
                    display_total_time = flash_buf_interval_time;
                    display_first_time = millis();
                    display_forever_flag = 0;
                    ledFlashTimes = LED_FLASH_COUNT+10;
                }else
                {
                    display_type = DISPLAY_NOTHING;
                }
            }
            else
            {   
                // scroll to next character
                if (((millis() - scroll_last_time) >= scroll_interval_time))
                {
                    display_stop_flag = true;
                    if ((display_list_index == 0) && (display_list_length > 1))
                    {
                        display_list_index = 1;
                        memset(display_buf, 0, 384);
                        writeDisplayBuf(display_list[0], display_color);
                        writeSpareBuf(display_list[1], display_color);
                        adjustDisplayBuf2();
                        scroll_mark = 0;
                    }
                    else
                    {
                        scroll_mark++;
                        if (scroll_mark > 8)   // it should be >7, but >8 looks more smoothly, why? scroll_mark?
                        {
                            scroll_mark = 0;
                            display_list_index++;
                            if (display_list_index < display_list_length)
                            {
                                scroll_mark++;
                                memset(display_buf, 0, 384);
                                writeDisplayBuf(display_list[display_list_index-1], display_color);
                                writeSpareBuf(display_list[display_list_index], display_color);
                                adjustDisplayBuf2();
                            }
                            else
                            {
                                display_list_index = 1;
                                memset(display_buf, 0, 384);
                                writeDisplayBuf(display_list[0], display_color);
                                writeSpareBuf(display_list[1], display_color);
                                adjustDisplayBuf2();
                            }
                        }
                    }
                    scroll_last_time = millis();
                    display_stop_flag = false;
                }
            }
        break;

        case DISPLAY_FLASH_BUFFER:      // display 64Byte custom pictures
            if (display_list_position < display_list_length)
            {
                if ((millis() - display_first_time) >= 2000)
                {// Lost packet, timeout.
                    display_type = DISPLAY_NOTHING;
                    display_list_position = 0;
                    display_list_length = 0;
                }
                break;
            }
            
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {
                // if time out, display nothing
                display_type = DISPLAY_NOTHING;
            }
            else
            {   // if only one picture
                if (flash_one_time_flag)
                {
                    display_stop_flag = true;
                    writeDisplayBuf(Flash.blockdata+48+16*flash_buf_start_pic_num);
                    adjustDisplayBuf();
                    flash_one_time_flag = false;
                    display_stop_flag = false;
                }
                else
                {
                    // if several pictures, switch display
                    if (((millis() - flash_buf_last_time) >= flash_buf_interval_time))
                    {
                        display_stop_flag = true;

                        writeDisplayBuf(Flash.blockdata+48+flash_buf_now_pic_num*16);
                        adjustDisplayBuf();
                        flash_buf_now_pic_num++;
                        if (flash_buf_now_pic_num > flash_buf_end_pic_num) flash_buf_now_pic_num = flash_buf_start_pic_num;

                        flash_buf_last_time = millis();
                        display_stop_flag = false;
                    }
                }
            }
        break;

        case DISPLAY_COLOR_BAR:
            // check if it is time out
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {   
                display_type = DISPLAY_NOTHING;
            }
            else
            {
                if (flash_one_time_flag)
                {
                    flash_one_time_flag = false;
                    display_stop_flag = true;
                    writeDisplayBufColorBarMode(display_list[0]);
                    adjustDisplayBuf();
                    display_stop_flag = false;
                }
            }
        break;

        case DISPLAY_COLOR_EMOJI:
            // check if it is time out
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {   
                display_type = DISPLAY_NOTHING;
            }
            else
            {
                if (flash_one_time_flag)
                {
                    flash_one_time_flag = false;
                    display_stop_flag = true;
                    writeDisplayBuf(((uint32_t *)COLORFUL_ICONS) + 16 * color_icons_index);
                    adjustDisplayBuf();
                    display_stop_flag = false;
                }
            }
        break;

        case DISPLAY_COLOR_WAVE_GIF:
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {   
                display_type = DISPLAY_NOTHING;
            }
            else
            {
                // scroll to next WAVE
                if (flash_one_time_flag)
                {
                    flash_one_time_flag = false;
                    display_stop_flag = true;
                    memset(display_buf, 0, 384);
                    writeDisplayBuf(WAVE, display_color);
                    writeSpareBuf(WAVE, display_color);
                    scroll_mark = 0;
                    display_stop_flag = false;
                }
                if (((millis() - scroll_last_time) >= scroll_interval_time))
                {
                    scroll_mark++;
                    if (scroll_mark > 7)
                    {
                        scroll_mark = 0;
                    }
                    scroll_last_time = millis();
                }
            }
        break;

        case DISPLAY_COLOR_CLOCKWISE_GIF:
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {   
                display_type = DISPLAY_NOTHING;
            }
            else
            {
                if (((millis() - flash_buf_last_time) >= flash_buf_interval_time))
                {
                    display_stop_flag = true;
                    if (clockwise_mode == ACW)
                    {
                        writeDisplayBuf(((uint32_t *)COLORFUL_FRAMES)+flash_buf_now_pic_num*16);
                        uint8_t orientation_temp = display_orientation;
                        display_orientation = DISPLAY_ROTATE_135;
                        adjustDisplayBuf();
                        display_orientation = orientation_temp;
                    }
                    else writeDisplayBuf(((uint32_t *)COLORFUL_FRAMES)+flash_buf_now_pic_num*16);
                    flash_buf_now_pic_num++;
                    if (flash_buf_now_pic_num > flash_buf_end_pic_num) flash_buf_now_pic_num = flash_buf_start_pic_num;            
                    flash_buf_last_time = millis();
                    display_stop_flag = false;                            
                }   
            }
        break;

        case DISPLAY_COLOR_NORMAL_GIF:
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {   
                display_type = DISPLAY_NOTHING;
            }
            else
            {
                if (flash_one_time_flag)
                {
                    display_stop_flag = true;
                    writeDisplayBuf(((uint32_t *)COLORFUL_FRAMES)+16*flash_buf_start_pic_num);
                    flash_one_time_flag = false;
                    display_stop_flag = false;
                }
                else
                {
                    // if several pictures, switch display
                    if (((millis() - flash_buf_last_time) >= flash_buf_interval_time))
                    {
                        display_stop_flag = true;

                        writeDisplayBuf(((uint32_t *)COLORFUL_FRAMES)+flash_buf_now_pic_num*16);
                        flash_buf_now_pic_num++;
                        if (flash_buf_now_pic_num > flash_buf_end_pic_num) flash_buf_now_pic_num = flash_buf_start_pic_num;

                        flash_buf_last_time = millis();
                        display_stop_flag = false;
                    }
                }
            }
        break;

        case DISPLAY_RAINBOW_CYCLE:
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {   
                display_type = DISPLAY_NOTHING;
            }
            else
            {
                if (((millis() - scroll_last_time) >= 30))
                {
                    display_stop_flag = true;
                    rainbowCycle();
                    scroll_last_time = millis();
                    display_stop_flag = false;
                }
            }
        break;

        case DISPLAY_FIRE:
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {   
                display_type = DISPLAY_NOTHING;
            }
            else
            {
                if (((millis() - scroll_last_time) >= 40))
                {
                    display_stop_flag = true;

                    fire();
                    scroll_last_time = millis();
                    display_stop_flag = false;
                }
            }
        break; 

        case DISPLAY_COLOR_BLOCK:
            if ((millis() - display_first_time >= display_total_time) && (display_forever_flag == false))
            {   
                display_type = DISPLAY_NOTHING;
            }
            else
            {
                if (flash_one_time_flag)
                {                
                    flash_one_time_flag = false;
                    // display_stop_flag = true;
                    colorBlock(color_block_value);
                    // display_stop_flag = false;
                }
            }
        break;   

        default:
        break;
    }
}


uint32_t wheel(uint8_t pos)
{
    // 0x00 red green blue
    uint32_t color = 0;

    if (pos < 42) color = ((GAMMA_2_2[255] << 16) + (GAMMA_2_2[pos * 6] << 8) + 0);
    else if (pos < 85)  { pos-=42; color = ((GAMMA_2_2[(255 - pos * 6)] << 16) + (GAMMA_2_2[(255)] << 8) + 0); }
    else if (pos < 128) { pos-=85; color = ((0) + ((GAMMA_2_2[255]) << 8) + (GAMMA_2_2[pos * 6])); }
    else if (pos < 170) { pos-=128; color = (0 + ((GAMMA_2_2[255 - pos * 6]) << 8) + GAMMA_2_2[255]); }
    else if (pos < 212) { pos-=170; color = (((GAMMA_2_2[pos * 6]) << 16) + 0 + GAMMA_2_2[255]); }
    else if (pos < 254) { pos-=212; color = ((GAMMA_2_2[255] << 16) + 0 + GAMMA_2_2[(255 - pos * 6)]); }
    else if (pos == 254) color = 0x00ffffff;
    else color = 0;
    return color;

}


void numberToStringList(int16_t num16)
{
    uint8_t length = 0;
    uint8_t data_length = 0;
    uint16_t positive_num16;
    // add a space to the display list at first

    if (num16 < 0)
    {
        positive_num16 = (uint16_t)((int32_t)(0 - num16));
        display_list[length] = ASCII[13];  // "-"
        length++;
    }
    else
    {
        positive_num16 = num16;
    }
    if (positive_num16 > 9999) data_length = 5;
    else if (positive_num16 > 999) data_length = 4;
    else if (positive_num16 > 99) data_length = 3;
    else if (positive_num16 > 9) data_length = 2;
    else data_length = 1;

    display_list_length = data_length + length;

    while (data_length)
    {
        display_list[length + data_length - 1] = ASCII[16 + positive_num16 % 10];
        positive_num16 /= 10;
        data_length--;
    }
}


void rainbowCycle(void)
{
    static uint8_t rainbow_count = 0;
    uint32_t rgb;
    for (uint8_t i = 0; i < 4; i++)
    {
        rgb = wheel((i*256/10+rainbow_count) % 254);
        // rgb = wheel((t+256/12)&255);
        for (uint8_t j=i; j < 8 - i; j++)
        {
            for (uint8_t k=i; k < 8 - i; k++)
            {
                display_buf[j][k][0] = (uint8_t)(rgb >> 16);
                display_buf[j][k][1] = (uint8_t)(rgb >> 8);
                display_buf[j][k][2] = (uint8_t)rgb;
            }
        }
    }

    rainbow_count++;
    rainbow_count++;
    // if (rainbow_count > 253) rainbow_count = 0;
}




void fire(void)
{
    if (fire_count >= 4)
    {

        // shift up
        for (uint8_t i=7;i>0;i--)
        {
            for (int8_t j=0;j<8;j++)
            {
                fire_value[i-1][j] = fire_value[i][j];
            }
        }
        for (uint8_t i=0;i<8;i++)
        {
            fire_value[7][i] = fire_line[i];
        }

        // get a line
        for (uint8_t i=0;i<8;i++)
        {
            if ((i == 3)||(i==4))
            {
                while(FIRE_RANDOMS[fire_count2]<8)
                {
                    fire_count2++;
                }
            }
            fire_line[i] = FIRE_RANDOMS[fire_count2];
            fire_count2++; 
        }

        fire_count = 0;
    }
    // draw frame
    // write to display buf
    for (uint8_t i=0;i < 8;i++){
        int16_t value =  ((int16_t)((10 - fire_count*3)*fire_value[7][i]) + (int16_t)((fire_count*3)*fire_line[i]))/10 - FIRE_MASK[7][i];
        value = max(0, value);
        display_buf[7][i][0] = (uint8_t)FIRE_COLORS[value][0];
        display_buf[7][i][1] = (uint8_t)FIRE_COLORS[value][1];
        display_buf[7][i][2] = (uint8_t)0;
    }

    for (uint8_t i=0; i < 7; i++)
    {
        for (uint8_t j=0; j < 8; j++)
        {
            int16_t value = ((int16_t)((10 - fire_count*3)*fire_value[i][j]) + (int16_t)((fire_count*3)*fire_value[i+1][j]))/10 - FIRE_MASK[i][j];
            value = max(0, value);
            display_buf[i][j][0] = (uint8_t)FIRE_COLORS[value][0];
            display_buf[i][j][1] = (uint8_t)FIRE_COLORS[value][1];
            display_buf[i][j][2] = (uint8_t)0;
        }
    }

    fire_count++;

}

void colorBlock(uint32_t color)
{
    uint8_t r = (uint8_t)(color >> 16);
    uint8_t g = (uint8_t)(color >> 8);
    uint8_t b = (uint8_t)(color >> 0);

    for (uint8_t i=0;i<8;i++)
    {
        for (uint8_t j=0;j<8;j++)
        {
            display_buf[i][j][0] = r;
            display_buf[i][j][1] = g;
            display_buf[i][j][2] = b;
        }
    }
}

