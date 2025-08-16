#ifndef BCC_H
#define BCC_H

#include <stdint.h>
#include <stdbool.h>

// Generate BCC for a buffer.
void bccGenerate(uint8_t *data_buffer, uint8_t size, uint8_t *xor);
// Generate a BCC for a buffer which starts with SOH character and compare with buffer's last character, which is BCC of buffer.
bool bccControl(uint8_t *buffer, uint8_t size);
// Generate a BCC and add it to end of a buffer. Buffer size should be 1 byte more than the buffer size.
void setBCC(uint8_t *buffer, uint8_t size, uint8_t xor);
// This function sends error message
void sendErrorMessage(char *error_text);
#endif
