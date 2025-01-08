//BLE Eddystone header files
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEBeacon.h>
#include <BLEAdvertising.h>
#include "freertos/ringbuf.h" //ring buffer header file
#include <DHT.h> //temp sensor header file

#define DHTPIN 18     // gpio pin to connect to sensor
#define DHTTYPE DHT11 

DHT dht(DHTPIN, DHTTYPE); //create dht object

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif


float temp_received; //global variable to receive temperature reading from sensor 


static SemaphoreHandle_t mutex; //create mutex

static QueueHandle_t msg_queue_temp = NULL; //create queue
static const uint8_t msg_queue_len = 5; //length of the queue

//Task Handles
static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;
static TaskHandle_t task_3 = NULL;

//Ring buffer initializations
RingbufHandle_t buf_handle;
int buffer_length = 5;
size_t buffer_size = buffer_length * sizeof(float) * 10;  // 5 * 4 = 20 bytes

//Eddystone
BLEAdvertising *pAdvertising;
uint32_t sequenceNumber = 0;

//Task 1: sensor read task which grabs a new temperature every second and puts that reading into a FreeRTOS Queue
void readTempandQueue(void *parameters){
  while (1){
    temp_received = dht.readTemperature();
    if (xQueueSend(msg_queue_temp, (void *)&temp_received, 0) != pdTRUE){ //if Queue is not full, send value
      Serial.println("QUEUE FULL");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); //delay for 1 second
  }
}

//Task 2: waits for a new reading in the FreeRTOS Queue and puts the new reading into a shared ring buffer and increments a FreeRTOS Semaphore.
void storeValueBuffer(void *parameters){
  float temp_buffer;
  while (1){
    if (xQueueReceive(msg_queue_temp, (void *)&temp_buffer, 0) == pdTRUE){
      //put in ring buffer if it is not full
      if (xRingbufferSend(buf_handle, &temp_buffer, sizeof(temp_buffer), 0) != pdTRUE) {
        Serial.println("RING BUFFER FULL");
      }
      xSemaphoreGive(mutex); //give mutex --> incremement count by 1 
    }
  }
}

//Task 3: checks the FreeRTOS Semaphore and updates the BLE eddystone beacontemperature data with the new reading if available.
void updateBLE(void *parameters){
  size_t item_size;

  while (1){
    if (xSemaphoreTake(mutex, 0) == pdTRUE){
      float *temp = (float *)xRingbufferReceive(buf_handle, &item_size, pdMS_TO_TICKS(1000));

      if (temp != NULL){
        float temp_ble = *temp;
        Serial.print("TEMP:   ");
        Serial.println(temp_ble);
        broadcastTemperature(temp_ble);
        vRingbufferReturnItem(buf_handle, (void*)temp); //to prevent a memory leak
      }
    }
  }
}
void loop() {
    vTaskDelay(100 / portTICK_PERIOD_MS); //delay for 1 second
}
void broadcastTemperature(float temperature) {
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  
  //Eddystone URL data and frame
  uint8_t eddyData[20];
  int i = 0;
  eddyData[i++] = 0x10;  // frame type
  eddyData[i++] = 0xF8;  // TX power at 0m
  eddyData[i++] = 0x03;  // URL prefix (https://)
  //add domain and temp
  String url_str = "temp.io/" + String(temperature, 1);
  memcpy(&eddyData[i], url_str.c_str(), url_str.length());
  i += url_str.length();
  //convert to string
  String serviceData = String((char*)eddyData, i);
  oAdvertisementData.setServiceData(BLEUUID((uint16_t)0xFEAA), serviceData);
  //sets flags
  oAdvertisementData.setFlags(0x06);  
  //start advertisement
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->start();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  dht.begin();
  //set up BLE device
  BLEDevice::init("TempBeacon");
  pAdvertising = BLEDevice::getAdvertising();

  //create queue 
  msg_queue_temp = xQueueCreate(msg_queue_len, sizeof(float));

  //intialize ring buffer
  buf_handle = xRingbufferCreate(
    buffer_size,                    // Size in bytes
    RINGBUF_TYPE_NOSPLIT    // Buffer type
  );
  //initialize mutex
  mutex = xSemaphoreCreateCounting(buffer_length, 0); //counting semaphore

  //Create Tasks
  //Task 1: sensor read task which grabs a new temperature every second and puts that reading into a FreeRTOS Queue
  //Task 2: waits for a new reading in the FreeRTOS Queue and puts the new reading into a shared ring buffer and increments a FreeRTOS Semaphore.
  //Task 3: checks the FreeRTOS Semaphore and updates the BLE eddystone beacontemperature data with the new reading if available.
  xTaskCreatePinnedToCore(
    readTempandQueue,            // Task function
    "print from Queue",         // Task name
    4096,                 // Stack size increased to 4096
    NULL,                 // Parameter
    1,  
    &task_1,                  // Task priority                 // Task handle
    app_cpu               // Run on one core (or tskNO_AFFINITY)
  );

  xTaskCreatePinnedToCore(
    storeValueBuffer,            // Task function
    "put in ring buffer",         // Task name
    4096,                 // Stack size increased to 4096
    NULL,                 // Parameter
    1,  
    &task_2,                  // Task priority                 // Task handle
    app_cpu               // Run on one core (or tskNO_AFFINITY)
  );


  xTaskCreatePinnedToCore(
    updateBLE,            // Task function
    "update value to BLE eddystone beacon",         // Task name
    4096,                 // Stack size increased to 4096
    NULL,                 // Parameter
    1,  
    &task_3,                  // Task priority                 // Task handle
    app_cpu               // Run on one core (or tskNO_AFFINITY)
  );
}

