/*
 * File:        utils.cpp
 * Project:     RocketMotorPIDGimbal V1.3
 * Author:      Boris du Reau
 * Description: Small formatting helpers shared by the telemetry and the
 *              flight data dump.
 */
#include "utils.h"

/*
   floatToByte()
   Write the 4 raw bytes of a float as 8 upper-case hexadecimal characters
   followed by a null terminator. The Android application decodes this
   format, so it must not be changed.
*/
void floatToByte(float f, char *ret) {
  byte * b = (byte *) &f;
  for (int i = 0; i < 4; i++) {

    byte b1 = (b[i] >> 4) & 0x0f;   // high nibble
    byte b2 = (b[i] & 0x0f);        // low nibble

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    ret[i * 2] = c1;
    ret[(i * 2) + 1] = c2;
  }
  ret[8] = '\0';
}
