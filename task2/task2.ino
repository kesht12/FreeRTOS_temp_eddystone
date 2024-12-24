//task 2: waits for a new reading in the FreeRTOS Queue and puts the new reading into a shared ring buffer and increments a FreeRTOS Semaphore.


#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

//Task 2: 
void setup() {

}

void loop() {

}
