#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Definição dos pinos para os LEDs
// Semáforo da direita
const uint8_t red_right = 25;
const uint8_t yellow_right = 33;
const uint8_t green_right = 32;

// Semáforo da esquerda
const uint8_t red_left = 15;
const uint8_t yellow_left = 16; // Alterado de 2 para 16
const uint8_t green_left = 4;   // Alterado de 0 para 4

// Pinos dos sensores
const uint8_t day_night_sensor_pin = 34;     // LDR para detecção de dia/noite
const uint8_t traffic_light_sensor_pin = 35; // LDR para detecção de veículos

// Limiares
const int threshold_day_night = 2000; // Ajustar com base em testes (valor maior indica dia)
const int threshold_vehicle_detected = 100; // Ajustar com base em testes (valor menor indica veículo detectado)

// Variáveis de temporização
unsigned long lastUpdateTime = 0;
int trafficLightState = 0; // 0: verde, 1: amarelo, 2: vermelho

unsigned long lastFlashTime = 0;
bool yellowOn = false;

// Durações em milissegundos
const unsigned long greenDuration = 5000;  // 5 segundos
const unsigned long yellowDuration = 2000; // 2 segundos
const unsigned long redDuration = 5000;    // 5 segundos

// Detalhes da conexão Wi-Fi
const char* ssid = "Inteli.Iot";
const char* password = "@Intelix10T#";

// Configurações MQTT Ubidots
const char* mqtt_server = "industrial.api.ubidots.com";
const int mqtt_port = 1883;
const char* mqtt_token = "BBUS-cFNi57frlHNpqzlbQrWTq4GzouX5mA"; // Substitua pelo seu token real
const char* client_id = "esp32_t12_g04";

// Tópicos MQTT
const char* topic_modo_noturno = "/v1.6/devices/semaforo/modo_noturno";
const char* topic_ldr = "/v1.6/devices/esp32_t12_g04/ldr_valor";
const char* topic_vehicle = "/v1.6/devices/semaforo/vehicle_detection";

WiFiClient espClient;
PubSubClient client(espClient);

// Função para leitura filtrada dos sensores
int readSensor(int pin) {
  int total = 0;
  const int samples = 10;
  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delay(5);
  }
  return total / samples;
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando-se à ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 10000) { // Timeout de 10 segundos
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("Wi-Fi conectado");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("Falha ao conectar ao Wi-Fi");
    // Implementar lógica de falha conforme necessário
  }
}

void reconnect() {
  // Loop até reconectar
  while (!client.connected()) {
    Serial.print("Tentando conectar ao MQTT...");
    // Tenta conectar
    if (client.connect(client_id, mqtt_token, "")) {
      Serial.println("Conectado!");
      // Se houver tópicos para subscrever, faça aqui
      // client.subscribe("algum/topico");
    } else {
      Serial.print("Falha, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      // Espera 5 segundos antes de tentar novamente
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);

  // Configurar pinos dos LEDs como saída
  pinMode(red_right, OUTPUT);
  pinMode(yellow_right, OUTPUT);
  pinMode(green_right, OUTPUT);

  pinMode(red_left, OUTPUT);
  pinMode(yellow_left, OUTPUT);
  pinMode(green_left, OUTPUT);

  // Configurar pinos dos sensores como entrada
  pinMode(day_night_sensor_pin, INPUT);
  pinMode(traffic_light_sensor_pin, INPUT);

  // Inicializar conexão Wi-Fi
  setup_wifi();

  // Configurar cliente MQTT
  client.setServer(mqtt_server, mqtt_port);
  // Se precisar de callbacks, defina aqui
  // client.setCallback(callback);

  // Inicializar temporização
  lastUpdateTime = millis();
  lastFlashTime = millis();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Ler valores dos sensores com filtragem
  int dayNightValue = readSensor(day_night_sensor_pin);
  int trafficLightValue = readSensor(traffic_light_sensor_pin);

  Serial.print("Valor do Sensor Dia/Noite: ");
  Serial.println(dayNightValue);

  Serial.print("Valor do Sensor de Detecção de Veículos: ");
  Serial.println(trafficLightValue);

  // Determinar modo dia/noite
  if (dayNightValue > threshold_day_night) {
    // Modo dia
    normalTrafficLightSequence();
  } else {
    // Modo noite
    nightModeSequence();
  }

  // Enviar dados para o Ubidots via MQTT
  sendToUbidots(dayNightValue, trafficLightValue);

  // Atualizar sem usar delay()
  // Aqui você pode adicionar outras tarefas não bloqueantes
}

void normalTrafficLightSequence() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastUpdateTime;

  // Ler o sensor de detecção de veículos com filtragem
  int trafficLightValue = readSensor(traffic_light_sensor_pin);
  bool vehicleDetected = trafficLightValue < threshold_vehicle_detected;

  switch (trafficLightState) {
    case 0: // Sinal verde
      if (elapsedTime >= greenDuration) {
        // Mudar para o estado amarelo
        setTrafficLightState(1);
        lastUpdateTime = currentTime;
      } else if (vehicleDetected) {
        // Opcionalmente, estender a duração do sinal verde
        // lastUpdateTime = currentTime; // Descomente esta linha para estender o sinal verde
      }
      break;
    case 1: // Sinal amarelo
      if (elapsedTime >= yellowDuration) {
        // Mudar para o estado vermelho
        setTrafficLightState(2);
        lastUpdateTime = currentTime;
      }
      break;
    case 2: // Sinal vermelho
      if (elapsedTime >= redDuration) {
        // Mudar para o estado verde
        setTrafficLightState(0);
        lastUpdateTime = currentTime;
      }
      break;
  }
}

void setTrafficLightState(int state) {
  trafficLightState = state;
  switch(state) {
    case 0: // Sinal verde no esquerdo, vermelho no direito
      // Semáforo esquerdo
      digitalWrite(green_left, HIGH);
      digitalWrite(yellow_left, LOW);
      digitalWrite(red_left, LOW);
      // Semáforo direito
      digitalWrite(green_right, LOW);
      digitalWrite(yellow_right, LOW);
      digitalWrite(red_right, HIGH);
      break;
    case 1: // Sinal amarelo no esquerdo, vermelho no direito
      // Semáforo esquerdo
      digitalWrite(green_left, LOW);
      digitalWrite(yellow_left, HIGH);
      digitalWrite(red_left, LOW);
      // Semáforo direito
      digitalWrite(green_right, LOW);
      digitalWrite(yellow_right, LOW);
      digitalWrite(red_right, HIGH);
      break;
    case 2: // Sinal vermelho no esquerdo, verde no direito
      // Semáforo esquerdo
      digitalWrite(green_left, LOW);
      digitalWrite(yellow_left, LOW);
      digitalWrite(red_left, HIGH);
      // Semáforo direito
      digitalWrite(green_right, HIGH);
      digitalWrite(yellow_right, LOW);
      digitalWrite(red_right, LOW);
      break;
  }
}

void nightModeSequence() {
  unsigned long currentTime = millis();
  if (currentTime - lastFlashTime >= 1000) { // Piscar a cada segundo
    lastFlashTime = currentTime;
    yellowOn = !yellowOn;
    if (yellowOn) {
      // Acender luzes amarelas
      // Semáforo esquerdo
      digitalWrite(green_left, LOW);
      digitalWrite(yellow_left, HIGH);
      digitalWrite(red_left, LOW);
      // Semáforo direito
      digitalWrite(green_right, LOW);
      digitalWrite(yellow_right, HIGH);
      digitalWrite(red_right, LOW);
    } else {
      // Apagar todas as luzes
      // Semáforo esquerdo
      digitalWrite(green_left, LOW);
      digitalWrite(yellow_left, LOW);
      digitalWrite(red_left, LOW);
      // Semáforo direito
      digitalWrite(green_right, LOW);
      digitalWrite(yellow_right, LOW);
      digitalWrite(red_right, LOW);
    }
  }
}

void sendToUbidots(int dayNightValue, int trafficLightValue) {
  if (client.connected()) {
    // Publicar modo noturno
    String modoNoturno = (dayNightValue > threshold_day_night) ? "dia" : "noite";
    if (modoNoturno == "noite") {
      client.publish(topic_modo_noturno, "1", true); // 1 para modo noturno
    } else {
      client.publish(topic_modo_noturno, "0", true); // 0 para modo diurno
    }

    // Publicar valor do LDR (luminosidade)
    String luminosidade = String(dayNightValue);
    client.publish(topic_ldr, luminosidade.c_str(), true);

    // Publicar valor da detecção de veículo
    String veiculo = String(trafficLightValue);
    client.publish(topic_vehicle, veiculo.c_str(), true);

    Serial.println("Dados enviados para o Ubidots via MQTT");
  } else {
    Serial.println("Falha na conexão MQTT");
  }
}
