#define BLYNK_TEMPLATE_ID "TMPL3DDQZ4bqq"
#define BLYNK_TEMPLATE_NAME "Heart Rate"
#define BLYNK_AUTH_TOKEN "********" //Provide Your BLYNK Template AuthToken

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <PulseSensorPlayground.h>

char ssid[] = "*****";              // Wi-Fi SSID
char pass[] = "*********";          // Wi-Fi password

const int pulsePin = 34;               // GPIO 34 on ESP32 for Pulse Sensor
const int physicalLEDPin = 2;          // GPIO 2 on ESP32 for physical LED
const int thresholdBPMHigh = 100;      // Heart rate threshold for high alert (above 100)
const int thresholdBPMLow = 30;        // Heart rate threshold for low alert (below 30)

// PulseSensor Playground object
PulseSensorPlayground pulseSensor;

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);

  // Set physical LED pin as output
  pinMode(physicalLEDPin, OUTPUT);

  // Connect to Wi-Fi and Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Configure PulseSensor object
  pulseSensor.analogInput(pulsePin);   // Set the analog input pin for pulse
  pulseSensor.setThreshold(550);       // Set sensor threshold (adjust as needed)

  // Start the pulse sensor
  if (pulseSensor.begin()) {
    Serial.println("Pulse sensor started successfully.");
  }
}

void loop() {
  // Run Blynk
  Blynk.run();

  // Read pulse sensor and calculate BPM
  int myBPM = pulseSensor.getBeatsPerMinute();

  // Check if a heartbeat was detected
  if (pulseSensor.sawStartOfBeat()) {
    Serial.print("Heartbeat detected! BPM: ");
    Serial.println(myBPM);

    // Send BPM to Blynk app on virtual pin V0
    Blynk.virtualWrite(V0, myBPM);

    // Check BPM and update both Blynk virtual LED and physical LED
    if (myBPM > thresholdBPMHigh || myBPM < thresholdBPMLow) {
      Blynk.virtualWrite(V1, 255);   // Turn on virtual LED at V1 for high or low BPM
      digitalWrite(physicalLEDPin, HIGH);  // Turn on physical LED
    } else {
      Blynk.virtualWrite(V1, 0);     // Turn off virtual LED at V1 for normal BPM
      digitalWrite(physicalLEDPin, LOW);   // Turn off physical LED
    }
  }

  // Delay for stability
  delay(20);
}
