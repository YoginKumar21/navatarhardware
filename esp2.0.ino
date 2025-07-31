#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// === WiFi Credentials ===
const char* ssid = "TP-Link_37BA";
const char* password = "99386320";

// === MQTT Broker ===
const char* mqtt_server = "a9088c6daa9e41e4b8f965ad7fd902a5.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "yogin";
const char* mqtt_pass = "Yogin@2004";

// === MQTT Topics ===
const char* mqtt_control_topic = "robot/control";
const char* mqtt_status_topic = "bot/status";

// === ESC Control Pins ===
#define ESC_LEFT_PIN 14
#define ESC_RIGHT_PIN 27

// === Ultrasonic Sensor Pins ===
#define TRIG_LEFT 17
#define ECHO_LEFT 16

#define TRIG_CENTER 5
#define ECHO_CENTER 18

#define TRIG_RIGHT 4
#define ECHO_RIGHT 19 // <-- Use safe GPIO, not 0

// Obstacle threshold in cm (1.5ft ≈ 45cm)
const int OBSTACLE_DISTANCE_CM = 45;

Servo escLeft;
Servo escRight;
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Store last PWM command
int lastLeftPWM = 1500;
int lastRightPWM = 1500;

// Utility: Read one ultrasonic sensor in cm
long readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  long distanceCm = (duration == 0 ? 999 : duration / 58);
  return distanceCm;
}

void setup_wifi() {
  Serial.print("Connecting to WiFi ");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
}

void setESCs(int leftPulse, int rightPulse) {
  leftPulse = constrain(leftPulse, 1000, 2000);
  rightPulse = constrain(rightPulse, 1000, 2000);
  escLeft.writeMicroseconds(leftPulse);
  escRight.writeMicroseconds(rightPulse);
  lastLeftPWM = leftPulse;
  lastRightPWM = rightPulse;
}

void stopESCs() {
  setESCs(1500,1500);
}

void softStopESCs() {
  int steps = 8;
  for (int i=1; i<=steps; i++) {
    int l = lastLeftPWM + (1500-lastLeftPWM) * i / steps;
    int r = lastRightPWM + (1500-lastRightPWM) * i / steps;
    escLeft.writeMicroseconds(l);
    escRight.writeMicroseconds(r);
    delay(20);
  }
  stopESCs();
}

void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("❌ JSON Parse Error: ");
    Serial.println(error.c_str());
    return;
  }

  String requestedDirection = doc["direction"];
  requestedDirection.toLowerCase();
  int speed = doc["speed"];

  // Always check for obstacles first
  long distLeft = readUltrasonicCM(TRIG_LEFT, ECHO_LEFT);
  long distCenter = readUltrasonicCM(TRIG_CENTER, ECHO_CENTER);
  long distRight = readUltrasonicCM(TRIG_RIGHT, ECHO_RIGHT);

  bool obstacleFront = distCenter <= OBSTACLE_DISTANCE_CM;
  bool obstacleLeft = distLeft <= OBSTACLE_DISTANCE_CM;
  bool obstacleRight = distRight <= OBSTACLE_DISTANCE_CM;

  String obstacleMsg = "";
  if (obstacleFront || obstacleLeft || obstacleRight) {
    obstacleMsg = "Obstacle detected at:";
    if (obstacleFront) obstacleMsg += " front";
    if (obstacleLeft) obstacleMsg += " left";
    if (obstacleRight) obstacleMsg += " right";
    client.publish(mqtt_status_topic, obstacleMsg.c_str());
  } else {
    client.publish(mqtt_status_topic, "No obstacles detected");
  }

  // Selectively block movement if obstacle is detected in direction requested
  bool blockMove = false;
  if (speed > 0) {
    if (obstacleFront && requestedDirection.startsWith("forward")) blockMove = true;
    if (obstacleLeft && requestedDirection.indexOf("left") >= 0) blockMove = true;
    if (obstacleRight && requestedDirection.indexOf("right") >= 0) blockMove = true;
  }

  if (blockMove) {
    Serial.println("🚫 Movement blocked due to obstacle");
    softStopESCs();
    client.publish(mqtt_status_topic, "movement blocked due to obstacle");
    return;
  }

  // Accept and execute command if not blocked
  speed = constrain(speed, 0, 30);

  int forwardPWM = map(speed, 0, 100, 1500, 2000);
  int reversePWM = map(speed, 0, 100, 1500, 1000);
  int diagPWM = map(speed, 0, 100, 1500, 1850);

  if (requestedDirection == "forward") {
    setESCs(forwardPWM, forwardPWM);
  } else if (requestedDirection == "backward") {
    setESCs(reversePWM, reversePWM);
  } else if (requestedDirection == "left") {
    setESCs(reversePWM, forwardPWM);
  } else if (requestedDirection == "right") {
    setESCs(forwardPWM, reversePWM);
  } else if (requestedDirection == "forward-right") {
    setESCs(forwardPWM, diagPWM);
  } else if (requestedDirection == "forward-left") {
    setESCs(diagPWM, forwardPWM);
  } else if (requestedDirection == "backward-right") {
    setESCs(reversePWM, map(speed, 0, 100, 1500, 1150));
  } else if (requestedDirection == "backward-left") {
    setESCs(map(speed, 0, 100, 1500, 1150), reversePWM);
  } else if (requestedDirection == "stop" || requestedDirection == "stopped") {
    softStopESCs();
  } else {
    stopESCs();
  }

  // Publish motion status if not blocked
  if (speed > 0 && requestedDirection != "stop" && requestedDirection != "stopped")
    client.publish(mqtt_status_topic, "moving");
  else
    client.publish(mqtt_status_topic, "stopped");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("🔄 Connecting to MQTT...");
    String clientId = "ESP32Client_" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected!");
      delay(100);
      if (client.subscribe(mqtt_control_topic)) {
        Serial.println("✅ Subscribed to control topic");
      } else {
        Serial.println("❌ Failed to subscribe");
      }
      client.publish(mqtt_status_topic, "connected");
      Serial.println("✅ Published status");
    } else {
      Serial.print("Failed. rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("🚀 Starting ESP32 ESC MQTT Bot...");

  // Setup ultrasonic sensor pins
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_CENTER, OUTPUT);
  pinMode(ECHO_CENTER, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  escLeft.setPeriodHertz(50);
  escRight.setPeriodHertz(50);
  escLeft.attach(ESC_LEFT_PIN, 1000, 2000);
  escRight.attach(ESC_RIGHT_PIN, 1000, 2000);
  delay(3000);
  stopESCs();

  setup_wifi();

  Serial.println("📡 Setting up MQTT...");
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
