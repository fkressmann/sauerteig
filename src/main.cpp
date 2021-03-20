#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <credentials.h>

ESP8266WiFiMulti wifiMulti; 

// Default Threshold Temperature Value
String inputMessage = "24.0";
String lastTemperature;
String enableArmChecked = "checked";
String inputMessage2 = "true";
float temp_is = 0;

struct {
  float threshold_lower = 23;
  float threshold_upper = 25;
  bool active = true;
} data;

// HTML web page to handle 2 input fields (threshold_input, enable_arm_input)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Sauerteig Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="10">
  </head><body>
  <h2>Aktuelle Temperatur</h2> 
  <h3>%TEMPERATURE% &deg;C</h3>
  <h3>%ACTIVE%</h3>
  <h2>Einstellungen</h2>
  <form action="/get">
    Einschalten ab <input type="number" step="0.1" name="threshold_input_lower" value="%THRESHOLD_LOWER%" required><br>
    Ausschalten ab <input type="number" step="0.1" name="threshold_input_upper" value="%THRESHOLD_UPPER%" required><br>
    Aktiv? <input type="checkbox" name="enable_arm_input" value="true" %ENABLE_ARM_INPUT%><br><br>
    <input type="submit" value="Speichern">
  </form>
</body></html>)rawliteral";

// Flag variable to keep track if triggers was activated or not
bool output = false;

const char* THRESHOLD_LOWER = "threshold_input_lower";
const char* THRESHOLD_UPPER = "threshold_input_upper";
const char* ACTIVE = "enable_arm_input";

// Interval between sensor readings. Learn more about ESP32 timers: https://RandomNerdTutorials.com/esp32-pir-motion-sensor-interrupts-timers/
unsigned long previousMillis = 0;     
const long interval = 5000;    

// GPIO where the output is connected to
const int out_pin = D1;
const int LED = D4;
// GPIO where the DS18B20 is connected to
const int oneWireBus = D3;     
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server(80);

// Replaces placeholder with DS18B20 values
String processor(const String& var){
  //Serial.println(var);
  if(var == "TEMPERATURE"){
    return String(temp_is);
  }
  else if(var == "THRESHOLD_LOWER"){
    return String(data.threshold_lower);
  }
  else if(var == "THRESHOLD_UPPER"){
    return String(data.threshold_upper);
  }
  else if(var == "ACTIVE"){
    return output ? "Heizung an" : "Heizung aus";
  }
  else if(var == "ENABLE_ARM_INPUT"){
    return data.active ? "checked" : "";
  }
  return String();
}

void print_data() {
  Serial.println("lower: " + String(data.threshold_lower));
  Serial.println("upper: " + String(data.threshold_upper));
  Serial.println("active?: " + String(data.active));
}

void setup() {
  Serial.begin(115200);
  WiFi.hostname("sauerteig");
  wifiMulti.addAP(STA_SSID_1, STA_PSK_1);
  wifiMulti.addAP(STA_SSID_2, STA_PSK_2);
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(1000);
    Serial.println("WiFi running...");
  }
  Serial.println();
  Serial.print("ESP IP Address: http://");
  Serial.println(WiFi.localIP());
  

  EEPROM.begin(sizeof(data));
  EEPROM.get(0, data);
  EEPROM.end();
  Serial.println("Initialized with");
  print_data();
  
  pinMode(out_pin, OUTPUT);
  pinMode(oneWireBus, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(out_pin, LOW);
  
  // Start the DS18B20 sensor
  sensors.begin();
  
  // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Receive an HTTP GET request at <ESP_IP>/get?threshold_input=<inputMessage>&enable_arm_input=<inputMessage2>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    // GET threshold_input value on <ESP_IP>/get?threshold_input=<inputMessage>
    if (request->hasParam(THRESHOLD_LOWER)) {
      data.threshold_lower = request->getParam(THRESHOLD_LOWER)->value().toFloat();
      data.threshold_upper = request->getParam(THRESHOLD_UPPER)->value().toFloat();
      // GET enable_arm_input value on <ESP_IP>/get?enable_arm_input=<inputMessage2>
      if (request->hasParam(ACTIVE)) {
        data.active = true;
      }
      else {
        data.active = false;
      }
      EEPROM.begin(sizeof(data));
      EEPROM.put(0, data);
      EEPROM.commit();
      EEPROM.end();
    }
    Serial.println("Saved values:");
    print_data();
    // request->send(200, "text/html", "HTTP GET request sent to your ESP.<br><a href=\"/\">Return to Home Page</a>");
    // request->send_P(200, "text/html", index_html, processor);
    request->redirect("/");
  });
  server.onNotFound(notFound);
  server.begin();
}

void loop() {
  unsigned long currentMillis = millis();
  sensors.requestTemperatures();
  temp_is = sensors.getTempCByIndex(0);
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if ( temp_is < -100 ) {
      Serial.println("No sensor connected, skipping");
      digitalWrite(out_pin, LOW);
      output = false;
      return;
    }
    Serial.print(temp_is);
    Serial.println(" *C");
    
    // Check if temperature is above threshold and if it needs to trigger output
    if(temp_is > data.threshold_upper && data.active && output){
      String message = "Temperature above threshold. Current temperature: " + 
                            String(temp_is) + "C";
      Serial.println(message);
      output = false;
      digitalWrite(out_pin, LOW);
    }
    // Check if temperature is below threshold and if it needs to trigger output
    else if(temp_is < data.threshold_lower && data.active && !output) {
      String message = "Temperature below threshold. Current temperature: " + 
                            String(temp_is) + " C";
      Serial.println(message);
      output = true;
      digitalWrite(out_pin, HIGH);
    }
    digitalWrite(LED, !digitalRead(LED));
  }
}