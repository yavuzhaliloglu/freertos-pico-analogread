#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>
#include <stdbool.h>
#include "project_globals.h" // ADC_FIFO struct tanımı için

void initADCFIFO(ADC_FIFO *f);
bool isFIFOFull(ADC_FIFO *f);
bool isFIFOEmpty(ADC_FIFO *f);
bool addToFIFO(ADC_FIFO *f, uint16_t data);
bool removeFromFIFO(ADC_FIFO *f);
void displayFIFO(ADC_FIFO *f);
bool removeFirstElementAddNewElement(ADC_FIFO *f, uint16_t data);
void getLastNElementsToBuffer(ADC_FIFO *f, uint16_t *buffer, uint16_t count);
#endif