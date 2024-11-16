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

  // if (iBusSerial.available()) {
  //     readiBusData();
  // }

  // int Roll_raw     = channelData[0];
  // int Pitch_raw    = channelData[1];
  // int Yaw_raw      = channelData[3];
  // int Throttle_raw = channelData[2];

  // int Roll        = map(constrain(Roll_raw,     1000, 2000), 1000, 2000, -254,  254);
  // int Pitch       = map(constrain(Pitch_raw,    1000, 2000), 1000, 2000, -254,  254);
  // int Yaw         = map(constrain(Yaw_raw,      1000, 2000), 1000, 2000, -254,  254);
  // int Throttle    = map(constrain(Throttle_raw, 1000, 2000), 1000, 2000, 0,  508);

  // // set deadzone
  // if (abs(Roll) < 24){
  //     Roll = 0;
  // }
  // if (abs(Pitch) < 24){
  //     Pitch = 0;
  // }
  // if (abs(Yaw) < 24){
  //     Yaw = 0;
  // }
  // if (abs(Throttle) < 24){
  //     Throttle = 0;
  // }

  // RPY_Setpoint[0] = Roll;
  // RPY_Setpoint[1] = Pitch;
  // RPY_Setpoint[2] = Yaw;
  
  // Serial.print("Roll: ");
  // Serial.print(Roll);
  // Serial.print(", Pitch: ");
  // Serial.print(Pitch);
  // Serial.print(", Yaw: ");
  // Serial.print(Yaw);
  // Serial.print(", Throttle: ");
  // Serial.print(Throttle);

  // Serial.print("Roll_Setpoint: ");
  // Serial.print(RPY_Setpoint[0]);
  // Serial.print(", Pitch_Setpoint: ");
  // Serial.print(RPY_Setpoint[1]);
  // Serial.print(", Yaw: ");
  // Serial.print(RPY_Setpoint[2]);
  // Serial.print(", Throttle_Setpoint: ");
  // Serial.println(Throttle);
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
