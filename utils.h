/*
 * File:        utils.h
 * Project:     RocketMotorPIDGimbal V1.3
 * Author:      Boris du Reau
 * Description: Small formatting helpers shared by the telemetry and the
 *              flight data dump.
 */
#ifndef _UTILS_H
#define _UTILS_H
#include "Arduino.h"

// Convert a float into its 8-character hexadecimal representation
// (raw IEEE-754 bytes in memory order). "ret" must hold at least 9 chars.
void floatToByte(float f, char *ret);

#endif
