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

struct {
  float temp_set = 24;
  int kp = 90;
  int ki = 30;
  int kd = 80;
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
    Temperatur Soll <input type="number" step="0.1" name="set_point" value="%set_point%" required><br>
    Kp <input type="number" step="1" name="kp" value="%kp%" required> P: %p%<br>
    Ki <input type="number" step="1" name="ki" value="%ki%" required> I: %i%<br>
    Kd <input type="number" step="1" name="kd" value="%kd%" required> D: %d%<br>
    Aktiv? <input type="checkbox" name="enable_arm_input" value="true" %ENABLE_ARM_INPUT%><br><br>
    <input type="submit" value="Speichern">
  </form>
</body></html>)rawliteral";

// Flag variable to keep track if triggers was activated or not
bool output = false;

const char* SET_POINT = "set_point";
const char* KP = "kp";
const char* KI = "ki";
const char* KD = "kd";
const char* ACTIVE = "enable_arm_input";

float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, currentTime, timePrev;
float PID_value = 0;
int button_pressed = 0;
int menu_activated=0;
float last_set_temperature = 0;

float PID_p = 0;    float PID_i = 0;    float PID_d = 0;
float last_kp = 0;  float last_ki = 0;  float last_kd = 0;
  

// GPIO where the output is connected to
const int out_pin = D5;
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
  if(var == "TEMPERATURE") {
    return String(temperature_read);
  }
  else if(var == SET_POINT){
    return String(data.temp_set);
  }
  else if(var == KP){
    return String(data.kp);
  }
  else if(var == KI){
    return String(data.ki);
  }
  else if(var == KD){
    return String(data.kd);
  }
  else if(var == "p"){
    return String(PID_p);
  }
  else if(var == "i"){
    return String(PID_i);
  }
  else if(var == "d"){
    return String(PID_d);
  }
  else if(var == "ENABLE_ARM_INPUT") {
    return data.active ? "checked" : "";
  }
  return String();
}

void print_data() {
  Serial.println("setpoint: " + String(data.temp_set));
  Serial.println("kp: " + String(data.kp));
  Serial.println("ki: " + String(data.ki));
  Serial.println("kd: " + String(data.kd));
  Serial.println("active?: " + String(data.active));
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
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
    if (request->hasParam(SET_POINT)) {
      data.temp_set = request->getParam(SET_POINT)->value().toFloat();
      data.kp = request->getParam(KP)->value().toInt();
      data.ki = request->getParam(KI)->value().toInt();
      data.kd = request->getParam(KD)->value().toInt();
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
  analogWriteFreq(100);
  currentTime = millis();
}

void loop() {
  wifiMulti.run();
  sensors.requestTemperatures();
  temperature_read = sensors.getTempCByIndex(0);
  if (temperature_read < 0) return;
  Serial.println("temp " + String(temperature_read));
  // calculate the error between the setpoint and the real value
  PID_error = data.temp_set - temperature_read;
  //Calculate the P value
  PID_p = data.kp * PID_error;
  Serial.println("P " + String(PID_p));

  // Calculate the I value
  PID_i = PID_i + (0.01 * data.ki * PID_error);
  Serial.println("I " + String(PID_i));

  // For derivative we need real time to calculate speed change rate
  timePrev = currentTime;                            // save previously used currentTime to prevTime
  currentTime = millis();                            // actual time read
  elapsedTime = (currentTime - timePrev) / 1000; 
  // Calculate D value
  PID_d = data.kd *((PID_error - previous_error)/elapsedTime);
  Serial.println("D " + String(PID_d));
  // Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;
  Serial.println(PID_value);
  // Limit range of PWM to Arduino nano 10bit PWM
  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 1023)  
  {    PID_value = 1023;  }
  analogWrite(out_pin,PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.
}