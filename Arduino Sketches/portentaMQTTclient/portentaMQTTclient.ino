#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiSSLClient.h>
#include <WiFiUdp.h>
//-------------------------------------------
const char* ssid = "LANdo Calrissian";
const char* password = "WlllBasement69!";
const char* mqtt_user = "goats";
const char* mqtt_pw = "SD19nAiyLh";
const char* mqtt_server = "98.245.161.191";
//-------------------------------------------

//-------------------------------------------
String device_id = "PORTENTA";
String description = "A random number";
String units = "Furlongs";
//-------------------------------------------

WiFiClient portClient;


void setup()
{
  // Initilize hardware:
  Serial.begin(115200);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDB, OUTPUT);


  // Connect to the WiFi network (see function below loop)
  connectToWiFi(ssid, password);
  // setupDateTime();
  digitalWrite(LEDR, LOW); // LED off

//  // Connect to MQTT Server
//  client.setServer(mqtt_server, 1883);
//  client.setCallback(callback);
//  Serial.print("Press button 0 to reconnect ");
//  printLine();
}

void loop()
{
//  if (digitalRead(BUTTON_PIN) == LOW)
//  { // Check if button has been pressed
//    while (digitalRead(BUTTON_PIN) == LOW)
//      ; // Wait for button to be released
//
//    digitalWrite(LED_PIN, HIGH); // Turn on LED
//    connectToWiFi(ssid, password);
//    digitalWrite(LED_PIN, LOW); // Turn off LED
//  }

//  delay(1000);
//  long datapoint = random(-10, 10);
//  String timestamp = DateTime.format("%a %b %d %H:%M:%S GMT %Y").c_str();
//  //Serial.println("LOCAL:    " + timestamp + " -> " + datapoint);
//
//  if (!client.connected()) {
//    reconnect();
//  }
//
//  // Convert the value to a char array
//  String msg = "{\"randomInt\":" + String(datapoint) + ",\"timestamp\":\"" + timestamp + "\"}";
//  char buf[256];
//
//  msg.toCharArray(buf, 256);
//
//  Serial.print("Sending: ");
//  Serial.println(msg);
//  client.publish("devices/" + device_id + "/sensorData", buf);
//
//  client.loop();
}

// --------- Helper Functions ---------------------------
void connectToWiFi(const char * ssid, const char * pwd) {
  int ledState = 0;
  digitalWrite(LEDB, HIGH);
  delay(1000);
  digitalWrite(LEDB, LOW);
  printLine();
  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    // Blink LED while we're connecting:
    digitalWrite(LEDR, ledState);
    ledState = (ledState + 1) % 2; // Flip ledState
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.print(WiFi.localIP());
  printLine();
}

void printLine() {
  Serial.println();
  for (int i = 0; i < 30; i++)
    Serial.print("-");
  Serial.println();
}
