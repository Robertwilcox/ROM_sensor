#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// UART for communication with FPGA
#define UART_TX_PIN 17
#define UART_RX_PIN 16
HardwareSerial FPGA(1); // Use UART1 for communication

// UUIDs for FPGA's BLE service and characteristics - Replace with actual UUIDs
#define SERVICE_UUID           "DF816183-E1F1-4BF9-8B7D-3B794FA03C03"
#define CHARACTERISTIC_UUID_TX "AE40D235-6AA7-4764-89AB-108562631979" // Server TX, Client RX
#define CHARACTERISTIC_UUID_RX "12345678-1234-5678-1234-56789abcdef0" // Server RX, Client TX

static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static std::string myDeviceName = "ESP32_BNO055_BLE";
static BLERemoteCharacteristic* pRemoteCharacteristicTX = nullptr; // For receiving notifications
static BLERemoteCharacteristic* pRemoteCharacteristicRX = nullptr; // For sending data

// The remote service we wish to connect to.
static BLEUUID serviceUUID(SERVICE_UUID);
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID(CHARACTERISTIC_UUID_RX);

void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  Serial.print("Notification: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)pData[i]);
  }
  Serial.println();
}
bool connectToServer(BLEAddress pAddress) {
    Serial.print("Forming a connection to ");
    Serial.println(pAddress.toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    // Connect to the remove BLE Server.
    pClient->connect(pAddress);
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our service");

   // Obtain references to both the TX and RX characteristics in the remote BLE server
    pRemoteCharacteristicTX = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID_TX));
    if (pRemoteCharacteristicTX == nullptr || !pRemoteCharacteristicTX->canNotify()) {
      Serial.println("Failed to find our TX characteristic or it cannot notify");
      return false;
    }

    pRemoteCharacteristicTX->registerForNotify(notifyCallback);

    if (pRemoteCharacteristicTX != nullptr) {
    pRemoteCharacteristicTX->registerForNotify(notifyCallback);
    }

    pRemoteCharacteristicRX = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID_RX));
    if (pRemoteCharacteristicRX == nullptr) {
      Serial.println("Failed to find our RX characteristic");
      return false;
    }

    Serial.println(" - Found both characteristics");

    connected = true;
    return true;
    }


class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // Check if this is the device we want to connect to
    if (advertisedDevice.getName() == myDeviceName) {
      Serial.println("Device found. Connecting!");
      BLEDevice::getScan()->stop();
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
      doScan = true;
    }
  }
};

class MyClientCallback: public BLECharacteristicCallbacks {
    void onNotify(BLECharacteristic* pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.print("Notification: ");
            for (auto c: value) {
                Serial.print(c);
            }
            Serial.println();
        }
    }
};


void setup() {
  Serial.begin(115200);
  FPGA.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN); // Initialize UART for FPGA communication
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Start scanning
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); // Active scan uses more power, but get results faster
  pBLEScan->start(30, false); // Scan for 30 seconds and then stop
}

void loop() {
  if (doConnect) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("Connected to the BLE Server.");
      doConnect = false; // Reset connection attempt flag
    } else {
      Serial.println("Failed to connect to the server; retrying...");
      doConnect = false; // Reset connection attempt flag
      doScan = true; // Set to rescan
    }
  }

  // Handling data transmission to and from the FPGA
  if (connected) {
    if (FPGA.available()) {
      String message = FPGA.readString();
      if (pRemoteCharacteristicRX->canWrite()) {
        pRemoteCharacteristicRX->writeValue(message.c_str(), message.length());
        Serial.println("Data sent to BLE device.");
      }
    }
  } else if (doScan) {
    BLEDevice::getScan()->start(0); // Continuous scan
    doScan = false;
  }

  delay(1000);
}