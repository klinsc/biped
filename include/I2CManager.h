#ifndef I2CMANAGER_H
#define I2CMANAGER_H

#include <Arduino.h>
#include <Wire.h>

class I2CManager {
public:
    static SemaphoreHandle_t mutex;

    static void begin() {
        mutex = xSemaphoreCreateMutex();
        Wire.begin(21, 22);
        Wire.setClock(400000);
    }

    class Lock {
    public:
        Lock() { xSemaphoreTake(I2CManager::mutex, portMAX_DELAY); }
        ~Lock() { xSemaphoreGive(I2CManager::mutex); }
    };
};

#endif
