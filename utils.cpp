#include "utils.h"

// ================================================================
// === Those 2 functions will format the data                   ===
// ================================================================

void floatToByte(float f, char *ret) {
  byte * b = (byte *) &f;
  for (int i = 0; i < 4; i++) {

    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    ret[i*2] = c1;
    ret[(i*2)+1] = c2;
  }
  ret[8]='\0';
}
