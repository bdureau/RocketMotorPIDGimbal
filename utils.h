#ifndef _UTILS_H
#define _UTILS_H
#include "Arduino.h"


void serialPrintFloatArr(float * arr, int length);
void serialFloatPrint(float f); 
void floatToByte(float f, char *ret);

#endif
