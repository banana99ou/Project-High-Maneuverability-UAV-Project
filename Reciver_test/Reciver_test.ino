#include <HardwareSerial.h>

HardwareSerial iBusSerial(2);  // Using UART2 (RX2 = GPIO 16, TX2 = GPIO 17)

#define NUM_CHANNELS 6 // Adjust based on the number of channels in your receiver

// iBus protocol specific constants
#define IBUS_HEADER 0x20
#define IBUS_LENGTH 0x16 // Total 32 bytes

// Buffer for iBus data
uint8_t ibusData[32];
uint16_t channelData[NUM_CHANNELS];

void setup() {
  Serial.begin(115200);  // Serial for debugging
  iBusSerial.begin(115200, SERIAL_8N1, 16, 17);  // iBus runs at 115200 baud rate, RX = 16, TX = 17

  Serial.println("iBus Reading Example - ESP32");
}

void loop() {
  if (iBusSerial.available()) {
    if (readiBusData()) {
      // Successfully read iBus data, now process channels
      for (int i = 0; i < NUM_CHANNELS; i++) {
        Serial.print("Channel ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(channelData[i]);
      }
      Serial.println();
    }
  }
}

// Function to read iBus data and store in channelData
bool readiBusData() {
  // Read the full 32-byte iBus packet
  if (iBusSerial.available() >= IBUS_LENGTH) {
    iBusSerial.readBytes(ibusData, IBUS_LENGTH);

    // Check if it's an iBus header
    if (ibusData[0] == IBUS_HEADER) {
      // Parse channel data from the packet
      for (int i = 0; i < NUM_CHANNELS; i++) {
        // Channels are 2 bytes each (low byte, high byte)
        channelData[i] = ibusData[2 + i * 2] | (ibusData[3 + i * 2] << 8);
      }
      return true;
    }
  }
  return false;
}
