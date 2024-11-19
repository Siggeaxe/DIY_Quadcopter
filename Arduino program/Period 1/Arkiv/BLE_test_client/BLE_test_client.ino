#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEAdvertisedDevice* myDevice;
BLEClient* pClient = nullptr; // Client to connect to the server
bool isConnected = false;      // Connection status

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("Advertised Device found: ");
        Serial.println(advertisedDevice.toString().c_str());

        // Check if the found device advertises the desired service
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
            Serial.println("Found our device! Connecting...");
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            advertisedDevice.getScan()->stop(); // Stop scanning once we find the device
        }
    }
};

void connectToServer() {
    if (myDevice == nullptr) return;

    pClient = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->connect(myDevice);  // Connect to server
    Serial.println(" - Connected to server");
    isConnected = true; // Update connection status

    BLERemoteService* pRemoteService = pClient->getService(BLEUUID(SERVICE_UUID));
    if (pRemoteService == nullptr) {
        Serial.print("Failed to find service UUID: ");
        Serial.println(SERVICE_UUID);
        pClient->disconnect();
        isConnected = false; // Update connection status
        return;
    }
    Serial.println(" - Found service");

    BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
    if (pRemoteCharacteristic == nullptr) {
        Serial.print("Failed to find characteristic UUID: ");
        Serial.println(CHARACTERISTIC_UUID);
        pClient->disconnect();
        isConnected = false; // Update connection status
        return;
    }
    Serial.println(" - Found characteristic");

    // Read and write in a loop while connected
    for (int i = 0; i < 5; i++) {  // You can adjust the number of iterations
        if (pRemoteCharacteristic->canWrite()) {
            String newValue = "Hello World from Client " + String(i);
            pRemoteCharacteristic->writeValue(newValue);
            Serial.println("Wrote new value: " + newValue);
        }

        if (pRemoteCharacteristic->canRead()) {
            String value = pRemoteCharacteristic->readValue();
            Serial.print("Characteristic value: ");
            Serial.println(value);
        }

        delay(2000);  // Wait 2 seconds between reads/writes
    }

    pClient->disconnect();
    Serial.println("Client disconnected");
    isConnected = false; // Update connection status
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE Client...");

    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
}

void loop() {
    // If not connected, start scanning
    if (!isConnected) {
        BLEScan* pBLEScan = BLEDevice::getScan();
        pBLEScan->start(5, false); // Scan for 5 seconds
        delay(1000); // Delay to prevent continuous scanning too fast
    } else {
        connectToServer(); // Call function to read/write if connected
    }
}
