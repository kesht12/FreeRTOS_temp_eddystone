# **Temperature Monitoring with BLE Eddystone Beacon**

A simple FreeRTOS-based project that reads temperature values from a DHT11 sensor and broadcasts them using Google's BLE Eddystone protocol.

## **Hardware**

1. ESP32-S3
2. DHT11 Temperature Sensor
3. Wires

## **Software**

1. Arduino IDE


## **Flow Chart**

![alt text](https://github.com/kesht12/esp32_project/blob/main/FlowChart.jpeg)


## **Concepts Used**
1. Queues: FreeRTOS queues for temporary storage of temperature values
2. Ring Buffers: Circular buffer to efficiently store and read data
3. Signal mechanisms: Counting semaphore for synchronization between data storage and BLE broadcasting
4. Task Management: Concurrently ran multiple FreeRTOS tasks
5. BLE beacon protocol: Used Google's eddystone protocol

## **Implementation of Tasks**

1. Places temperature readings from DHT11 sensor in FreeRTOS queue every second.
2. When data is available in queue, it is transferred into a ring buffer. Uses a counting semaphore to indicate that there is new data in the buffer.  
3. If semaphore is succesfully acquired, update the BLE Eddystone beacon with newest temperature reading.







 
