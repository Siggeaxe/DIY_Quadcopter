
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

#define ESPNOW_WIFI_CHANNEL 6


class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  // Constructor of the class using the broadcast address
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Broadcast_Peer() {
    remove();
  }

  // Function to properly initialize the ESP-NOW and register the broadcast peer
  bool begin() {
    if (!ESP_NOW.begin() || !add()) {
      log_e("Failed to initialize ESP-NOW or register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to send a message to all devices within the network
  bool send_message(const uint8_t *data, size_t len) {
    if (!send(data, len)) {
      log_e("Failed to broadcast message");
      return false;
    }
    return true;
  }
};


uint32_t msg_count = 0;


ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL); // Create a broadcast peer object

/* Main */

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Register the broadcast peer
  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer");
    Serial.println("Reebooting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Setup complete.");
}

void loop() {
  // Broadcast a number instead of a string
  int V_A1 = analogRead(1);
  Serial.println(V_A1);
  uint8_t valueByte = V_A1 >> 4; //Throw away 4 lsb so that adc reading is 8 bit
  uint8_t commandByte = 0b00000001; //to change PCOM value
  uint16_t data = (commandByte << 8) | valueByte;

  // Use %u to print unsigned integers
  Serial.println("Broadcasting message:");
  Serial.print(data, BIN);

  // Send the number as raw data
  if (!broadcast_peer.send_message((uint8_t *)&data, sizeof(data))) {
    Serial.println("Failed to broadcast message");
  }

  delay(100);
}

