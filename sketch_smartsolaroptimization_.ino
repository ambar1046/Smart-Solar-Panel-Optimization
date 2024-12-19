#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Blynk credentials
char auth[] = "YourBlynkAuthToken";
char ssid[] = "YourWiFiSSID";
char pass[] = "YourWiFiPassword";

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pins configuration
const int voltageSensorPin = ;  // Voltage sensor pin
const int tempSensorPin = ;      // DS18B20 data pin
const int relayPin = ;           // Relay control pin
const int buzzerPin = ;          // Buzzer control pin

// Voltage and temperature thresholds
const float lowVoltageThreshold = 5.0;  // Adjust based on battery specs
const float normalVoltageThreshold = 12.0; //reference for small solar panel max voltage is 12V
const float highTempThreshold = 50.0;    // Adjust as needed

// DS18B20 setup
OneWire oneWire(tempSensorPin);
DallasTemperature sensors(&oneWire);

void setup() {
  // Initialize Serial monitor
  Serial.begin(115200);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize pins
  pinMode(relayPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  digitalWrite(buzzerPin, LOW);

  // Initialize DS18B20 sensor
  sensors.begin();

  // Connect to Wi-Fi and Blynk
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  Blynk.begin(auth, ssid, pass);
}

void loop() {
  Blynk.run();

  // Read voltage and temperature values
  float voltage = readVoltage();
  float temperature = readTemperature();

  // Determine battery level percentage
  int batteryLevel = map(voltage, 0, normalVoltageThreshold, 0, 100);
  if (batteryLevel > 100) batteryLevel = 100;

  // Display values on LCD
  lcd.setCursor(0, 0);
  lcd.print("V: ");
  lcd.print(voltage, 1);
  lcd.print("V T: ");
  lcd.print(temperature, 1);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Batt: ");
  lcd.print(batteryLevel);
  lcd.print("%");

  // Send data to Blynk
  Blynk.virtualWrite(V0, voltage);
  Blynk.virtualWrite(V1, temperature);
  Blynk.virtualWrite(V2, batteryLevel);

  // Control relay & buzzer based on conditions
  if (temperature > highTempThreshold && voltage < lowVoltageThreshold) {
    digitalWrite(relayPin, HIGH);
    digitalWrite(buzzerPin, HIGH);
  } else if (voltage >= normalVoltageThreshold) {
    digitalWrite(relayPin, LOW);
    digitalWrite(buzzerPin, LOW);
  }
  delay(1000); // Update every second
}

float readVoltage() {
  int sensorValue = analogRead(voltageSensorPin);
  float voltage = sensorValue * (5.0 / 1023.0) * 11; // Adjust for voltage divider ratio
  return voltage;
}

float readTemperature() {
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);
  if (temperature == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: DS18B20 disconnected!");
    return 0.0; // Return 0.0 if there's an error
  }
  return temperature;
}
