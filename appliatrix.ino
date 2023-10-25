#include <WiFi.h>
#include <PubSubClient.h>
#include <WifiLocation.h>
#define DEVICE_ID "device9" // Replace with your device ID
#define WIFI_SSID "TARIMO" // Replace with your WiFi SSID
#define WIFI_PASSWORD "swatch07" // Replace with your WiFi password

const char* mqttServer = "40.76.95.237";
const int mqttPort = 1883;
const char* mqttUsername = "admin" ;
const char* mqttPassword = "123Give!@#";
const char* googleApiKey = "AIzaSyAJRUccXQaN3rL9HeFe0JqQp67VRAoFUmM" ;

// Pin definitions
const int currentSensorPin = 33; // GPIO 33 for current sensor
const int relayPin = 14; // GPIO 14 for relay
const int relayReset = 2;
const int relayEnable = 15 ;

// Power data variables
float current;
float power;
float accumulatedKwh = 0.0 ;
String status = "ON";
unsigned long seconds = 0 ;

// WiFi and MQTT client
WiFiClient wifiClient;
PubSubClient mqttClient (wifiClient);
WifiLocation location (googleApiKey);
unsigned long lastSampleTime = 0 ;
unsigned long lastSendTime = 0 ;
const unsigned long sampleInterval = 1000 ; // Sample data every 1 second
const unsigned long sendInterval = 60000 ; // Send data every 1 minute
bool wifiConnected = false ;

void setClock () {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8*3600*2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
}
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("\n");
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}

void setup () {
  Serial.begin(115200);
  pinMode(relayPin,  OUTPUT);
  pinMode(relayEnable, OUTPUT);
  pinMode(relayReset, OUTPUT);
  analogReadResolution(12);

// Connect to Wi-Fi

  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while(WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
}
  Serial.println("Connected to WiFi");

// Set up MQTT
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);

// Connect to MQTT broker
  while(!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if(mqttClient.connect(DEVICE_ID, mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT");
      mqttClient.subscribe(String("/down/" + String(DEVICE_ID)).c_str());
      Rset();
}
    else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println("Retrying in 5 seconds...");
      delay(5000);
}
}
}

void loop () {
  mqttClient.loop();
  unsigned long currentMillis = millis();

// Sample power data every 1 second
  if(currentMillis - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentMillis;
    samplePowerData();
}

// Send data every 1 minute
  if(currentMillis - lastSendTime >= sendInterval) {
    if ( WiFi.status() != WL_CONNECTED){
      lastSampleTime = currentMillis;
}
    else {
      lastSendTime = currentMillis;
      publishData();
      resetAccumulatedData();
}
}
}

void samplePowerData () {

// Read current data from current sensor
  float AmpsRMS, wattage;
  float result;
  float kwh = 0 ;
  delay(10);
  float readValue = analogRead(currentSensorPin);
  if(readValue == 0) {
    result = 0 ;
}
  else if(readValue < 1520 && readValue > 1480) {
    result = 0 ;
}
  else {
    result = abs(1.5 - ((readValue * (3.3/4096)) + 0.3));
}
  AmpsRMS = abs((result/0.066));
  wattage = (AmpsRMS * 222)/ 1000 ;
  kwh = wattage/3600;
  Serial.println("Current: " + String(AmpsRMS));
  Serial.println("KiloWatts: " + String(wattage));
  Serial.println("Consumption: " + String(kwh));
  Serial.println("Analogue Value: " + String(readValue));
  Serial.println("Seconds: " + String(seconds));
  accumulatedKwh = accumulatedKwh + kwh;
  seconds++;
}

void publishData () {
  setClock();
  location_t loc = location.getGeoFromWiFi();

// Create JSON payload
  String payload = "{\"deviceID\":\"" + String (DEVICE_ID) + "\",\"consumption\":\"" + String(accumulatedKwh, 4) + "\",\"location\":\"" + String(loc.lat, 7) + "," + String(loc.lon, 7) + "\",\"Seconds\":\"" + String(seconds, 1 ) + "\",\"status\":\"" + String(status) + "\"}" ;
  Serial.println("Payload: " + payload);

//Publish data to MQTT topic
  String topic = "/up/" + String(DEVICE_ID);
  Serial.println("Topic: " + topic);
  mqttClient.publish(topic.c_str(), payload.c_str());
}

void resetAccumulatedData () {
  accumulatedKwh = 0.0;
  seconds = 0;
}

void callback(char* topic, byte* payload, unsigned int length) {
  String receivedTopic = String(topic);
  String receivedMessage = "";

// Concatenate the payload bytes into a string
  for(int i = 0; i < length; i++) {
    receivedMessage += (char)payload[i];
}
  Serial.print("Received message: ");
  Serial.println(receivedMessage);
  if(receivedTopic.equals("/down/" + String(DEVICE_ID))) {

// Check the received message and control the relay accordingly
  if(receivedMessage.equals("OFF")) {
    R_reset();
    status = "OFF" ;
}
  else if(receivedMessage.equals("ON")) {
    Rset();
    status = "ON";
}
  Serial.println("Relay state updated");
}
}

void Rset () {
  Serial.println("Set");
  digitalWrite(relayEnable,HIGH);
  delay(500);
  digitalWrite(relayPin,LOW);
  digitalWrite(relayReset,HIGH);
  delay(500);
  digitalWrite(relayEnable,LOW);
}

void R_reset () {
  Serial.println("reset");
  digitalWrite(relayEnable,HIGH);
  delay(500);
  digitalWrite(relayPin,HIGH);
  digitalWrite(relayReset,LOW);
  delay(500);
  digitalWrite(relayEnable,LOW);
}

void mqttConnect () {
  while(!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if(mqttClient.connect(DEVICE_ID, mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT");
      mqttClient.subscribe(String("/down/" + String(DEVICE_ID)).c_str());
      R_reset();
}
    else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println("Retrying in 5 seconds...");
      delay(5000);
}
}
}

void connectToWiFi () {
  int connectionAttempts = 0;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Reconnecting to WiFi..." );
  while(WiFi.status() != WL_CONNECTED && connectionAttempts < 10 ) {
    delay(1000);
    Serial.print(".");
    connectionAttempts++;
}
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nConnected to WiFi");
}
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
  mqttConnect();
}
