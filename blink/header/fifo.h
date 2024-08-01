void initADCFIFO(ADC_FIFO *f)
{
    f->head = 0;
    f->tail = 0;
    f->count = 0;
}

bool isFIFOFull(ADC_FIFO *f)
{
    return (f->count == ADC_FIFO_SIZE);
}

bool isFIFOEmpty(ADC_FIFO *f)
{
    return (f->count == 0);
}

bool addToFIFO(ADC_FIFO *f, uint16_t data)
{
    if (isFIFOFull(f))
    {
        // PRINTF("ADDTOFIFO: FIFO is full. cannot add %u to FIFO!\n", data);
        return false;
    }
    f->data[f->tail] = data;
    f->tail = (f->tail + 1) % ADC_FIFO_SIZE;
    f->count++;
    return true;
}

bool removeFromFIFO(ADC_FIFO *f)
{
    if (isFIFOEmpty(f))
    {
        // PRINTF("REMOVEFROMFIFO: FIFO is empty. cannot remove from FIFO!\n");
        return false;
    }
    f->head = (f->head + 1) % ADC_FIFO_SIZE;
    f->count--;
    return true;
}

void displayFIFO(ADC_FIFO *f)
{
    PRINTF("ADCFIFO: head = %u, tail = %u, count = %u. Content of FIFO is:", f->head, f->tail, f->count);
    for (int fifo_idx = 0; fifo_idx < f->count; fifo_idx++)
    {
        if (fifo_idx % 10 == 0)
        {
            PRINTF("\n");
        }

        if (fifo_idx % 100 == 0)
        {
            PRINTF("\n");
        }

        PRINTF("%u ", f->data[(f->head + fifo_idx) % ADC_FIFO_SIZE]);
    }
    PRINTF("\n");
}

void displayFIFOStats(ADC_FIFO *f)
{
    PRINTF("ADCFIFO: head = %u, tail = %u, count = %u.\n", f->head, f->tail, f->count);
}

bool removeFirstElementAddNewElement(ADC_FIFO *f, uint16_t data)
{
    bool is_removed_from_fifo = removeFromFIFO(f);
    if (is_removed_from_fifo)
    {
        bool is_added_to_fifo = addToFIFO(f, data);
        if (is_added_to_fifo)
        {
            return true;
        }
    }

    return false;
}

// Function to get the last N elements of the FIFO
void getLastNElementsToBuffer(ADC_FIFO *f, uint16_t *buffer, uint16_t count)
{
    if (count > f->count)
    {
        count = f->count;
    }
    for (uint16_t i = 0; i < count; i++)
    {
        buffer[i] = f->data[(f->tail - count + i + ADC_FIFO_SIZE) % ADC_FIFO_SIZE];
    }
}