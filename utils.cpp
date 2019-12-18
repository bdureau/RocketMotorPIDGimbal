#include "utils.h"

// ================================================================
// === Those 2 functions will format the data                   ===
// ================================================================
void serialPrintFloatArr(float * arr, int length) {
  for (int i = 0; i < length; i++) {
    serialFloatPrint(arr[i]);
    Serial1.print(",");
  }
}


void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for (int i = 0; i < 4; i++) {

    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    Serial1.print(c1);
    Serial1.print(c2);
  }
}
void floatToByte(float f, char *ret) {
  byte * b = (byte *) &f;
  for (int i = 0; i < 4; i++) {

    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    ret[0] = c1;
    ret[1] = c2;
  }
}
