#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "header/project_globals.h"

// UART Initialization
uint8_t initUART();
void sendResetDates();
void sendDeviceInfo();
// This function check the data which comes when State is Listening, and compares the message to defined strings, and returns a ListeningState value to process the request
enum ListeningStates checkListeningData(uint8_t *data_buffer, uint8_t size);
// This function deletes a character from a given string
void deleteChar(uint8_t *str, uint8_t len, char chr);
// This function gets the load profile data and finds the date characters and add them to time arrays
void parseLoadProfileDates(uint8_t *buffer, uint8_t len, uint8_t *reading_state_start_time, uint8_t *reading_state_end_time);
void parseThresholdRequestDates(uint8_t *buffer, uint8_t *start_date_inc, uint8_t *end_date_inc);
void parseACRequestDate(uint8_t *buffer, uint8_t *start_date, uint8_t *end_date);
uint8_t is_message_reset_factory_settings_message(uint8_t *msg_buf, uint8_t msg_len);
uint8_t is_message_reboot_device_message(uint8_t *msg_buf, uint8_t msg_len);
void reboot_device();
void reset_to_factory_settings();
uint8_t is_end_connection_message(uint8_t *msg_buf);
// This function gets the baud rate number like 1-6
uint8_t getProgramBaudRate(uint16_t b_rate);
// This function sets the device's baud rate according to given number like 0,1,2,3,4,5,6
void set_device_baud_rate(uint8_t b_rate);
// This function sets state to Greeting and resets rx_buffer and it's len. Also it sets the baud rate to 300, which is initial baud rate.
void reset_uart();
uint8_t verifyHourMinSec(uint8_t hour, uint8_t min, uint8_t sec);
uint8_t verifyYearMonthDay(uint8_t year, uint8_t month, uint8_t day);
// This function sets time via UART
void setTimeFromUART(uint8_t *buffer);
// This function sets date via UART
void setDateFromUART(uint8_t *buffer);
// This function generates a product,on info message and sends it to UART
void sendProductionInfo();
// This function gets a password and controls the password, if password is true, device sends an ACK message, if not, device sends NACK message
void passwordHandler(uint8_t *buffer);
void __not_in_flash_func(setThresholdValue)(uint8_t *data);
void getThresholdRecord(uint8_t *reading_state_start_time, uint8_t *reading_state_end_time, enum ListeningStates state);
void resetThresholdPIN();
void setThresholdPIN();
void getSuddenAmplitudeChangeRecords(uint8_t *reading_state_start_time, uint8_t *reading_state_end_time, enum ListeningStates state);
void readTime();
void readDate();
void readSerialNumber();
void sendLastVRMSXValue(enum ListeningStates vrmsState);
void sendThresholdObis();
bool control_serial_number(uint8_t *identification_req_buf, size_t req_size);
size_t create_identify_response_message(char *response_buf, size_t buf_size);
uint8_t exract_baud_rate_and_mode_from_message(uint8_t *msg_buf, size_t msg_len, int8_t *requested_mode);
void send_readout_message();
void set_init_baud_rate();
void send_programming_acknowledgement();
uint8_t is_message_break_command(uint8_t *buf);
#endif
