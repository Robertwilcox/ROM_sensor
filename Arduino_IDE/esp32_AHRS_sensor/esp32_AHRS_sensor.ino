#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // BNO055 uses address 0x28 by default, adjust if necessary

// BLE Service and Characteristic UUIDs - replace these with your generated UUIDs
#define SERVICE_UUID        "DF816183-E1F1-4BF9-8B7D-3B794FA03C03"
#define CHARACTERISTIC_UUID_TX "AE40D235-6AA7-4764-89AB-108562631979" // TX for transmitting data
#define CHARACTERISTIC_UUID_RX "12345678-1234-5678-1234-56789abcdef0" // RX for receiving data


BLECharacteristic *pCharacteristicTX; // For sending data
BLECharacteristic *pCharacteristicRX; // For receiving data

// Callback for receiving data
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        Serial.println("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        Serial.println();
      }
    }
};

void setup() {
  Serial.begin(115200);

  BLEDevice::init("ESP32_BNO055_BLE");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Characteristic for sending data
  pCharacteristicTX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_READ | 
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  // Characteristic for receiving data
  pCharacteristicRX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                      );

  // Assign the callback to the characteristic for receiving data
  pCharacteristicRX->setCallbacks(new MyCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("BLE service started, broadcasting...");
}

void loop() {
  // Send some test data every 1 second
  const char* testData = "Test data";
  pCharacteristicTX->setValue(testData);
  pCharacteristicTX->notify();


  delay(1000);
}
