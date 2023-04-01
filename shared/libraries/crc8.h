#pragma once

#include <stdint.h>
#include <string.h>

/**
 * CRC8 checksum calculation
 *
 * @param  pchMessage Message to check
 * @param  dwLength   Length of the message
 * @param  ucCRC8     Initialized checksum
 * @return            CRC checksum
 */
uint8_t get_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8);

/**
 * Verify CRC8
 *
 * @param  pchMessage Message to verify
 * @param  dwLength   Length = Data + Checksum
 * @return            1 for true, 0 for false
 */
uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);

/**
 * Append CRC8 to the end of message
 *
 * @param  pchMessage Message to calculate CRC and append
 * @param  dwLength   Length = Data + Checksum
 */
void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);