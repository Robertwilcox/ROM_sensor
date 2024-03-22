/**
 * @file main.cpp
 * @brief This file includes the setup and main loop for a BLE (Bluetooth Low Energy) client application using ESP32.
 *        It establishes a BLE connection to a server, in this case a specifically named device, to transmit and receive data.
 *        The application is designed to communicate with an FPGA through UART, receiving data from the BLE server
 *        and forwarding it to the FPGA. It also supports sending data back to the BLE server. The code includes
 *        initialization of BLE and UART, scanning for BLE devices, connecting to a specified BLE server using its
 *        UUIDs for services and characteristics, and handling data transmission.
 *
 *        UUIDs for the BLE service and its characteristics are defined for communication. Callback functions are
 *        implemented to handle BLE notifications and data forwarding. The main loop manages the connection state
 *        and data transmission between the ESP32 and the FPGA over UART. 
 *
 * @note Replace the UUIDs with those of your BLE server device. Ensure that the UART pins used match your hardware
 *       configuration. Adjust the device name and BLE scanning settings as required for your specific use case.
 */
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

static BLEAddress *pServerAddress; // Pointer to store the BLE server's address
static boolean doConnect = false; // Flag to initiate connection to the server
static boolean connected = false; // Flag to indicate connection status
static boolean doScan = false; // Flag to initiate scanning for BLE devices
static std::string myDeviceName = "ESP32_BNO055_BLE"; // Name of the device to connect to
static BLERemoteCharacteristic* pRemoteCharacteristicTX = nullptr; // Pointer to the characteristic for receiving notifications
static BLERemoteCharacteristic* pRemoteCharacteristicRX = nullptr; // Pointer to the characteristic for sending data

// The remote service we wish to connect to.
static BLEUUID serviceUUID(SERVICE_UUID);
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID(CHARACTERISTIC_UUID_RX);

// Global buffer and delimiter
String dataBuffer = "";
const char delimiter = '\n'; // Delimiter to indicate end of a message

// Callback function for processing notifications received from the BLE server
void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.print("Forwarding ");
    Serial.print(length);
    Serial.println(" bytes of data to the FPGA...");

    // Forward the raw binary data directly to the FPGA's UART
    FPGA.write(pData, length);

    // For debugging, print the raw data to the Serial monitor in hexadecimal format
    Serial.print("Data: ");
    for (size_t i = 0; i < length; i++) {
        Serial.print(pData[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

// Attempts to connect to the BLE server using its address
bool connectToServer(BLEAddress pAddress) {
    Serial.print("Forming a connection to ");
    Serial.println(pAddress.toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    // Connect to the remote BLE Server.
    pClient->connect(pAddress);
    Serial.println(" - Connected to server");

    // Attempt to discover the service we are interested in on the remote server
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

    // Register for notifications on the TX characteristic
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

// Custom callbacks for handling discovered BLE devices during scanning
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

// Defines a class to handle notifications from the BLE server's characteristics.
class MyClientCallback: public BLECharacteristicCallbacks {

    void onNotify(BLECharacteristic* pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
          // Print the notification
            Serial.print("Notification: ");
            for (auto c: value) {
                Serial.print(c);
            }
            // Newline for readability
            Serial.println();
        }
    }
};

// Setup[ function]
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

// Main loop
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

  delay(100);
}
