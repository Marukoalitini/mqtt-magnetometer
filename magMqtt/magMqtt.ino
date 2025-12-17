#include <WiFi.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <PubSubClient.h>

#define SD_CS_PIN 5

// Calibração inicial (substituir conforme necessário)
float B[3] { -182.17,  -16.58,   37.38};
float Ainv [3][3] {
  {  2.14764,  0.00263,  0.01469},
  {  0.00263,  2.08653, -0.00635},
  {  0.01469, -0.00635,  2.08382}
};

float fictitiousNorth = 0.0;
const float magneticDeclination = -20.45;
int ultimaDirecao = 0;

typedef struct vector { float x, y, z; } vector;

QMC5883LCompass compass;

// WiFi
const char* ssid = "NOME-DA-REDE";
const char* password = "SENHA";

// MQTT
const char* mqtt_server = "IP OU URL DO SERVIDOR MQTT";
const uint16_t mqtt_port = 1883; // Porta do servidor MQTT
const char* mqtt_client_id = "esp32-magnetometer";

// Tópicos
const char* topic_publish = "mag";
const char* topic_set_north = "mag/set-north";
const char* topic_set_calibration = "mag/set-calibration";
const char* topic_calibrate = "mag/calibrate";
const char* topic_calibration_data = "mag/calibration-data";
const char* topic_responses = "mag/response";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastPublish = 0;
const unsigned long publishInterval = 100UL;

// ---------------- Lidar com Cartão Sd ----------------
void saveFictitiousNorthToSD() {
  File file = SD.open("/fictitiousNorth.txt", FILE_WRITE);
  if (file) {
    file.println(fictitiousNorth, 6);
    file.close();
  }
}

void readFictitiousNorthFromSD() {
  File file = SD.open("/fictitiousNorth.txt", FILE_READ);
  if (file) {
    String northData = file.readStringUntil('\n');
    fictitiousNorth = northData.toFloat();
    file.close();
  }
}

void saveCalibrationData() {
  File file = SD.open("/calibration.txt", FILE_WRITE);
  if (file) {
    file.print("B:");
    for (int i = 0; i < 3; i++) {
      file.print(B[i], 6);
      if (i < 2) file.print(",");
    }
    file.println();
    file.print("Ainv:");
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        file.print(Ainv[i][j], 6);
        if (!(i == 2 && j == 2)) file.print(",");
      }
    }
    file.println();
    file.close();
  }
}

void loadCalibrationData() {
  File file = SD.open("/calibration.txt");
  if (!file) return;
  String line = file.readStringUntil('\n');
  if (line.startsWith("B:")) {
    line.remove(0, 2);
    int i = 0;
    while (line.length() > 0 && i < 3) {
      int idx = line.indexOf(',');
      if (idx < 0) idx = line.length();
      B[i++] = line.substring(0, idx).toFloat();
      if (idx >= (int)line.length()) break;
      line.remove(0, idx + 1);
    }
  }
  line = file.readStringUntil('\n');
  if (line.startsWith("Ainv:")) {
    line.remove(0, 5);
    int i = 0, j = 0;
    while (line.length() > 0 && i < 3) {
      int idx = line.indexOf(',');
      if (idx < 0) idx = line.length();
      Ainv[i][j++] = line.substring(0, idx).toFloat();
      if (j == 3) { i++; j = 0; }
      if (idx >= (int)line.length()) break;
      line.remove(0, idx + 1);
    }
  }
  file.close();
}

void read_data(vector * m) {
  compass.read();
  float x = compass.getX() - B[0];
  float y = compass.getY() - B[1];
  float z = compass.getZ() - B[2];
  m->x = Ainv[0][0] * x + Ainv[0][1] * y + Ainv[0][2] * z;
  m->y = Ainv[1][0] * x + Ainv[1][1] * y + Ainv[1][2] * z;
  m->z = Ainv[2][0] * x + Ainv[2][1]  * y + Ainv[2][2] * z;
}

String getMagnetometerJSON() {
  vector m;
  read_data(&m);
  int angulo = (int)(180. * atan2(m.y, m.x) / PI);
  angulo += (int)magneticDeclination;
  if (angulo < 0) angulo += 360;
  angulo -= (int)fictitiousNorth;
  if (angulo < 0) angulo += 360;
  int direcao;
  if ((angulo >= 0 && angulo < 35) || (angulo >= 325 && angulo < 360)) direcao = 0;
  else if (angulo >= 55 && angulo < 125) direcao = 3;
  else if (angulo >= 145 && angulo < 215) direcao = 2;
  else if (angulo >= 235 && angulo < 305) direcao = 1;
  else direcao = ultimaDirecao;
  ultimaDirecao = direcao;
  bool connected = WiFi.status() == WL_CONNECTED;
  StaticJsonDocument<256> doc;
  doc["angle"] = angulo;
  doc["direction"] = direcao;
  String json;
  serializeJson(doc, json);
  return json;
}

// ---------------- calibração ----------------
void calibrate() {
  unsigned long start = millis();

  // 1) Avisa para limpar dados antigos
  mqttClient.publish(topic_calibration_data, "START_CALIB");

  while (millis() - start < 30000UL) {
    delay(50);
    compass.read();
    int16_t rawX = compass.getX();
    int16_t rawY = compass.getY();
    int16_t rawZ = compass.getZ();

    // monta a linha
    String line = String(rawX) + "," + String(rawY) + "," + String(rawZ);

    // 2) envia ***linha por linha***
    mqttClient.publish(topic_calibration_data, line.c_str());

    Serial.println(line);
  }
  mqttClient.publish(topic_responses, "END_CALIB");

  Serial.println("Calibração enviada linha a linha!");
}

// ---------------- MQTT callbacks ----------------
void setFictitiousNorth() {
  vector m;
  read_data(&m);
  fictitiousNorth = 180. * atan2(m.y, m.x) / PI;
  fictitiousNorth += magneticDeclination;
  if (fictitiousNorth < 0) fictitiousNorth += 360;
  saveFictitiousNorthToSD();
  StaticJsonDocument<128> doc;
  doc["fictitiousNorth"] = fictitiousNorth;
  String json; serializeJson(doc, json);
  mqttClient.publish(topic_responses, json.c_str());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  String t = String(topic);
  if (t == topic_set_north) {
    if (msg == "CURRENT") {
      setFictitiousNorth();
    } else {
      float v = msg.toFloat();
      fictitiousNorth = v;
      saveFictitiousNorthToSD();
      StaticJsonDocument<128> doc;
      doc["fictitiousNorth"] = fictitiousNorth;
      String json; serializeJson(doc, json);
      mqttClient.publish(topic_responses, json.c_str());
    }
  } else if (t == topic_set_calibration) {
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, msg);
    if (!err) {
      JsonArray B_array = doc["B"].as<JsonArray>();
      for (int i = 0; i < 3; i++) B[i] = B_array[i];
      JsonArray Ainv_array = doc["Ainv"].as<JsonArray>();
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          Ainv[i][j] = Ainv_array[i][j];
      saveCalibrationData();
      mqttClient.publish(topic_responses, "calibration_updated");
    } else {
      mqttClient.publish(topic_responses, "calibration_error");
    }
  } else if (t == topic_calibrate) {
    calibrate();
    mqttClient.publish(topic_responses, "calibration_done");
  }
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect(mqtt_client_id)) {
      mqttClient.subscribe(topic_set_north);
      mqttClient.subscribe(topic_set_calibration);
      mqttClient.subscribe(topic_calibrate);
    } else {
      delay(2000);
    }
  }
}

void publishMagnetometer() {
  String json = getMagnetometerJSON();
  Serial.println("Mandando dados:" + json);
  mqttClient.publish(topic_publish, json.c_str());
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Erro ao acessar cartão SD");
  } else {
    Serial.println("Lendo dados do cartão SD");
    loadCalibrationData();
    readFictitiousNorthFromSD();
  }

  compass.init();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(".");
  Serial.println("Conectado");

  vector m;
  read_data(&m);
  //calcula o ângulo
  int angulo = 180. * atan2(m.y, m.x) / PI;
  angulo += magneticDeclination; //contabiliza a declinação magnética
  if (angulo < 0) angulo += 360;
  angulo -= fictitiousNorth;  // Ajusta com base no norte fictício
  if (angulo < 0) angulo += 360;
  //identifica em qial direção que o sensor está posicionado baseado no ângulo
  int direcao;
  if ((angulo >= 0 && angulo < 45) || (angulo >= 315 && angulo < 360)) {
      ultimaDirecao = 2;// 0 = Norte
  } else if (angulo >= 45 && angulo < 135) {
      ultimaDirecao = 3; // 1 = Leste
  } else if (angulo >= 135 && angulo < 225) {
      ultimaDirecao = 0; // 2 = Sul
  } else {
      ultimaDirecao = 1; // 3 = Oeste
  } 
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  
  lastPublish = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    delay(1000);
  }

  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  if (millis() - lastPublish >= publishInterval) {
    lastPublish = millis();
    publishMagnetometer();
  }
}
