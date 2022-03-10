#include <DateTime.h>
#include <DateTimeTZ.h>
#include <TimeElapsed.h>
#include <WiFi.h>
#include <PubSubClient.h>

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
PubSubClient client(portClient);
long lastMsg = 0;
// char msg[200];
int value = 0;

const int BUTTON_PIN = 0;
const int LED_PIN = 13;

void setup()
{
  // Initilize hardware:
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Connect to the WiFi network (see function below loop)
  connectToWiFi(ssid, password);
  setupDateTime();
  digitalWrite(LED_PIN, LOW); // LED off

  // Connect to MQTT Server
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Serial.print("Press button 0 to reconnect ");
  printLine();
}

void loop()
{
  if (digitalRead(BUTTON_PIN) == LOW)
  { // Check if button has been pressed
    while (digitalRead(BUTTON_PIN) == LOW)
      ; // Wait for button to be released

    digitalWrite(LED_PIN, HIGH); // Turn on LED
    connectToWiFi(ssid, password);
    digitalWrite(LED_PIN, LOW); // Turn off LED
  }

  delay(1000);
  long datapoint = hallRead(); //random(-10, 10);
  String timestamp = DateTime.format("%a %b %d %H:%M:%S GMT %Y").c_str();
  //Serial.println("LOCAL:    " + timestamp + " -> " + datapoint);

  if (!client.connected()) {
    reconnect();
  }
 // send nessage 1
  // Convert the value to a char array
  String msg = "{\"randomInt\":" + String(datapoint) + ",\"timestamp\":\"" + timestamp + "\"}";
  char buf[256];

  msg.toCharArray(buf, 256);

  Serial.print("Sending: ");
  Serial.println(msg);
  client.publish("devices/" + device_id + "/sensorData", buf);

//  // Send Message 2
//  // Convert the value to a char array
//   "xAcc": random.randrange(25, 35, 1),
//            "yAcc": random.randrange(25, 35, 1),
//            "zAcc": random.randrange(25, 35, 1),
//            "alphaAcc": random.randrange(25, 35, 1),
//            "betaAcc": random.randrange(25, 35, 1),
//            "omegaAcc": random.randrange(25, 35, 1),
//  String msg = "{\"xAcc\":" + String(random(-7,7) + ",\"yAcc\":" + String(random(-7,7)) + ",\"zAcc\":" + String(random(-7,7)) + ",\"alphaAcc\":" + String(random(-3,3)) + ",\"betaAcc\":" + String(random(-3,3)) + ",\"omegaAcc\":" + String(random(-3,3)) + "\",\"timestamp\":\"" + timestamp + "\"}";
//  char buf[256];
//
//  msg.toCharArray(buf, 256);
//
//  Serial.print("Sending: ");
//  Serial.println(msg);
//  client.publish("devices/deviceEmulator01/sensorData", buf);

  client.loop();
}

/* Helper functions*/
void setupDateTime() {
  // setup this after wifi connected
  // you can use custom timeZone,server and timeout
  // DateTime.setTimeZone(-4);
  //   DateTime.setServer("asia.pool.ntp.org");
  //   DateTime.begin(15 * 1000);
  DateTime.setServer("time.pool.aliyun.com");
  DateTime.setTimeZone("UTC+0");
  DateTime.begin();
  if (!DateTime.isTimeValid()) {
    Serial.println("Failed to get time from server.");
  } else {
    Serial.printf("Date Now is %s\n", DateTime.toISOString().c_str());
    Serial.printf("Timestamp is %ld\n", DateTime.now());
  }
}

void connectToWiFi(const char * ssid, const char * pwd) {
  int ledState = 0;
  printLine();
  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    // Blink LED while we're connecting:
    digitalWrite(LED_PIN, ledState);
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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("PortentaClient", mqtt_user, mqtt_pw)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("on");
      digitalWrite(LED_PIN, HIGH);
    }
    else if (messageTemp == "off") {
      Serial.println("off");
      digitalWrite(LED_PIN, LOW);
    }
  }
}
void printLine() {
  Serial.println();
  for (int i = 0; i < 30; i++)
    Serial.print("-");
  Serial.println();
}
