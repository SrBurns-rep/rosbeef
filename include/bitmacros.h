#ifndef __BITMACROS_H
#define __BITMACROS_H

/**
 * BIT is the index of a bit in a type
 * To set a bit use SETBIT
 * To clear a bit use CLEARBIT
 * To toggle a bit use FLIPBIT
 * To check if a bit is active use CHECKBIT
*/
#define CLEARBIT(TYPE, BIT)      (TYPE) &= ~(1 << (BIT))
#define SETBIT(TYPE, BIT)        (TYPE) |=  (1 << (BIT))
#define FLIPBIT(TYPE, BIT)       (TYPE) ^=  (1 << (BIT))
#define CHECKBIT(TYPE, BIT)      (TYPE) &   (1 << (BIT))

/**
 * Byte specific operations (char, unsigned char, ...)
*/
#define SWAPNIBBLES(BYTE)        (BYTE) = (BYTE)<<4 | (BYTE)>>4
#define INVERTBITORDER(BYTE)     {\
                                    (BYTE) = ((BYTE) << 4) & 0xf0 | ((BYTE) >> 4) & 0x0f;\
                                    (BYTE) = ((BYTE) & 0b00110011) << 2 | ((BYTE) & 0b11001100) >> 2;\
                                    (BYTE) = ((BYTE) & 0b01010101) << 1 | ((BYTE) & 0b10101010) >> 1;\
                                 }

/**
 * The mask size must be equal or smaller than the type,
 * it's safer to use a type with the same size
*/
#define CLEARMASK(TYPE, MASK)    (TYPE) & ~(MASK)
#define SETMASK(TYPE, MASK)      (TYPE) |  (MASK)
#define FLIPMASK(TYPE, MASK)     (TYPE) ^  (MASK)
#define CHECKMASK(TYPE, MASK)    (TYPE) &  (MASK)

#endif