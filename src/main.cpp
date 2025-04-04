#include <Arduino.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>

// Розмір EEPROM (залежить від вашого мікроконтролера)
#define EEPROM_SIZE 512

// Адреси для зберігання даних в EEPROM
#define EEPROM_ADDR_MODE_SET_HEAT 0
#define EEPROM_ADDR_TEMP_SETPOINT 10
#define EEPROM_ADDR_MODE_NASOS_BOY 20
#define EEPROM_ADDR_MODE_NASOS_HEAT 30
#define EEPROM_ADDR_HYSTERESIS 40

// Змінні для зберігання даних
bool modeSetHeat = false;
float tempSetpoint = 0.0;
bool modeNasosBoy = false;
bool modeNasosHeat = false;
float hysteresis = 0.5; // Значення за замовчуванням
float smoothedValue = 0;     // Поточне згладжене значення

// Константи для MQTT топіків
const char* TOPIC_MODE_SET_HEAT = "home/set/heat_on/mode/set";
const char* TOPIC_TEMP_SETPOINT = "home/set/heat_on/setpoint-temperature/get";
const char* TOPIC_MODE_NASOS_BOY = "home/set/boy_on/mode/nasos";
const char* TOPIC_MODE_NASOS_HEAT = "home/set/heat_on/mode/nasos";
const char* TOPIC_HYSTERESIS = "home/set/heat_on/setpoint-gisteresis/get";

// Клас для роботи з сенсором DS18B20
class TemperatureSensor {
private:
    OneWire oneWire;
    DallasTemperature sensors;
    unsigned long measurementInterval;
    unsigned long previousMillis;
    
public:
    // Конструктор
    TemperatureSensor(uint8_t pin, unsigned long interval)
        : oneWire(pin), sensors(&oneWire), measurementInterval(interval), previousMillis(0) {}

    // Ініціалізація сенсора
    void begin() {
        sensors.begin();
    }

    // Перевірка, чи настав час для нового вимірювання
    bool isTimeToMeasure() {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= measurementInterval) {
            previousMillis = currentMillis;
            return true;
        }
        return false;
    }

    // Отримання температури
    float getTemperature() {
        sensors.requestTemperatures();
        float tempC = sensors.getTempCByIndex(0);
  smoothedValue = tempC;
  smoothedValue = 0.9 * smoothedValue + 0.1 * tempC;
  // Check if reading was successful
  if (tempC != DEVICE_DISCONNECTED_C)
  {
    // Serial.print("Temperature for the device 1 (index 0) is: ");
    // Serial.println(tempC);
  }
  else
  {
    Serial.println("Error: Could not read temperature data");
  }
  return smoothedValue;
    }
};

// Клас для роботи з MQTT
class MQTTConnection {
private:
    WiFiClient espClient;
    PubSubClient client;
    const char* server;
    int port;
    const char* user;
    const char* password;

    // MQTT topics
    const char* TEMP_IN = "home/esp-12f/current-temperature";
    const char* HUM_IN = "home/esp-12f/current-humidity";
    const char* ESP_IP = "home/esp-12f/ip";
    const char* RELLAY = "home/esp-12f/heater";

public:
    // Конструктор
    MQTTConnection(const char* mqttServer, int mqttPort, const char* mqttUser, const char* mqttPass)
        : client(espClient), server(mqttServer), port(mqttPort), user(mqttUser), password(mqttPass) {}

    // Ініціалізація MQTT
    void begin() {
        client.setServer(server, port);
    }

    // Підключення до MQTT-сервера
    void connect() {
        while (!client.connected()) {
            Serial.print("Connecting to MQTT...");
            if (client.connect("ESP8266Client", user, password)) {
                Serial.println("connected");

                // Підписка на необхідні топіки
                client.subscribe("home/set/#");
                Serial.println("Subscribed to topics.");
            } else {
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" try again in 5 seconds");
                delay(5000);
            }
        }
    }
    void reconnectIfNeeded() {
        if (!client.connected()) {
            Serial.println("MQTT disconnected. Trying to reconnect...");
            connect(); // Вже є метод connect(), який виконує підписку та логін
        }
    }

    // Перевірка підключення
    bool isConnected() {
        return client.connected();
    }

    // Публікація повідомлення
    void publish(const char* topic, const char* message) {
        if (isConnected()) {
            client.publish(topic, message);
        }
    }

    // Обробка MQTT
    void loop() {
        client.loop();
    }

    // Методи для отримання топіків
    const char* getTempInTopic() { return TEMP_IN; }
    const char* getRelayTopic() { return RELLAY; }

    // Встановлення обробника вхідних повідомлень
    void setCallback(MQTT_CALLBACK_SIGNATURE) {
        client.setCallback(callback);
    }

    // Підписка на топік
    void subscribe(const char* topic) {
        client.subscribe(topic);
    }
};

// Клас для роботи з реле
class Relay {
private:
    uint8_t pin;
    bool state; // Змінна для зберігання поточного стану реле
    MQTTConnection& mqtt; // Посилання на об'єкт MQTTConnection

public:
    // Конструктор
    Relay(uint8_t relayPin, MQTTConnection& mqttConnection) 
        : pin(relayPin), state(false), mqtt(mqttConnection) {}

    // Ініціалізація реле
    void begin() {
        pinMode(pin, OUTPUT);
        off(); // Вимкнути реле за замовчуванням
    }

    // Увімкнути реле
    void on() {
        if (!state) { // Якщо реле ще не увімкнене
            digitalWrite(pin, HIGH);
            state = true;
            mqtt.publish(mqtt.getRelayTopic(), "ON"); // Публікація стану
            Serial.println("Relay ON: Published to MQTT.");
        }
    }

    // Вимкнути реле
    void off() {
        if (state) { // Якщо реле ще не вимкнене
            digitalWrite(pin, LOW);
            state = false;
            mqtt.publish(mqtt.getRelayTopic(), "OFF"); // Публікація стану
            Serial.println("Relay OFF: Published to MQTT.");
        }
    }

    // Отримати поточний стан реле
    bool getState() const {
        return state;
    }
};

// Клас для підключення до Wi-Fi
class WiFiConnection {
private:
    const char* ssid;
    const char* password;

public:
    // Конструктор
    WiFiConnection(const char* wifiSsid, const char* wifiPassword)
        : ssid(wifiSsid), password(wifiPassword) {}

    // Підключення до Wi-Fi
    void connect() {
        Serial.print("Connecting to Wi-Fi: ");
        Serial.println(ssid);

        WiFi.begin(ssid, password);

        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }

        Serial.println("\nWi-Fi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }
    void reconnectIfNeeded() {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected. Reconnecting...");
            WiFi.disconnect();
            WiFi.begin(ssid, password);
            unsigned long startAttemptTime = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
                delay(500);
                Serial.print(".");
            }
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\nWiFi reconnected!");
            } else {
                Serial.println("\nFailed to reconnect to WiFi.");
            }
        }
    }

    // Перевірка підключення до Wi-Fi
    bool isConnected() {
        return WiFi.status() == WL_CONNECTED;
    }
};

// Клас для логіки управління
class LogicController {
private:
    Relay& relay;
    TemperatureSensor& tempSensor;
    float hysteresis;

public:
    // Конструктор
    LogicController(Relay& relay, TemperatureSensor& tempSensor, float hysteresis)
        : relay(relay), tempSensor(tempSensor), hysteresis(hysteresis) {}

    // Основна логіка
    void update(bool modeSetHeat, bool modeNasosHeat, bool modeNasosBoy, float tempSetpoint) {
        Serial.println("LogicController::update called");
        Serial.println("modeNasosBoy: " + String(modeNasosBoy));

        float currentTemp = tempSensor.getTemperature();

        // Логіка для modeSetHeat і modeNasosHeat
        if (modeNasosHeat) {
            if (currentTemp <= (tempSetpoint - hysteresis)) {
                relay.on();
                Serial.println("Relay ON: Heating mode active and temperature below setpoint with hysteresis.");
            } else if (currentTemp >= tempSetpoint + hysteresis/2) {
                relay.off();
                Serial.println("Relay OFF: Temperature reached setpoint.");
            }
        } else if (modeNasosBoy) {
            relay.on();
            Serial.println("Relay ON: modeNasosBoy is active.");
            return;
        } else {
            relay.off();
            Serial.println("Relay OFF: Heating mode or pump mode is inactive.");
        }
    }
};

// MQTT налаштування
const char* mqtt_server = "greenhouse.net.ua";
const int mqtt_port = 1883;
const char* mqtt_user = "mqtt";
const char* mqtt_pass = "qwerty";

// Wi-Fi налаштування
const char* WIFI_SSID = "aonline";
const char* WIFI_PASSWORD = "1qaz2wsx3edc";

// Піни для підключення
#define ONE_WIRE_BUS D4
#define RELAY_PIN D3

// Створення об'єктів
TemperatureSensor tempSensor(ONE_WIRE_BUS, 1000);
WiFiConnection wifi(WIFI_SSID, WIFI_PASSWORD);
MQTTConnection mqtt(mqtt_server, mqtt_port, mqtt_user, mqtt_pass);
Relay relay(RELAY_PIN, mqtt);
LogicController logic(relay, tempSensor, 0.5); // 0.5 — значення гістерезису

// Функція для оновлення EEPROM
template <typename T>
void updateEEPROM(int address, const T& value) {
    EEPROM.put(address, value);
    EEPROM.commit();
}

// Обробник вхідних MQTT-повідомлень
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';
    String message = (char*)payload;

    Serial.println("Received MQTT message:");
    Serial.println("Topic: " + String(topic));
    Serial.println("Message: " + message);

    if (strcmp(topic, TOPIC_MODE_SET_HEAT) == 0) {
        modeSetHeat = (message == "heat");
        updateEEPROM(EEPROM_ADDR_MODE_SET_HEAT, modeSetHeat);
        Serial.println("Mode Set Heat updated: " + String(modeSetHeat));
    } else if (strcmp(topic, TOPIC_TEMP_SETPOINT) == 0) {
        tempSetpoint = message.toFloat();
        updateEEPROM(EEPROM_ADDR_TEMP_SETPOINT, tempSetpoint);
        Serial.println("Temperature Setpoint updated: " + String(tempSetpoint));
    } else if (strcmp(topic, TOPIC_MODE_NASOS_BOY) == 0) {
        modeNasosBoy = (message == "on");
        updateEEPROM(EEPROM_ADDR_MODE_NASOS_BOY, modeNasosBoy);
        Serial.println("Mode Nasos Boy updated: " + String(modeNasosBoy));
    } else if (strcmp(topic, TOPIC_MODE_NASOS_HEAT) == 0) {
        modeNasosHeat = (message == "1");
        updateEEPROM(EEPROM_ADDR_MODE_NASOS_HEAT, modeNasosHeat);
        Serial.println("Mode Nasos Heat updated: " + String(modeNasosHeat));
    } else if (strcmp(topic, TOPIC_HYSTERESIS) == 0) {
        hysteresis = message.toFloat();
        updateEEPROM(EEPROM_ADDR_HYSTERESIS, hysteresis);
        Serial.println("Hysteresis updated: " + String(hysteresis));
    } else {
        Serial.println("Unknown topic received.");
    }
}

void setup() {
    // Ініціалізація серійного монітора
    Serial.begin(9600);

    // Ініціалізація EEPROM
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(EEPROM_ADDR_MODE_SET_HEAT, modeSetHeat);
    EEPROM.get(EEPROM_ADDR_TEMP_SETPOINT, tempSetpoint);
    EEPROM.get(EEPROM_ADDR_MODE_NASOS_BOY, modeNasosBoy);
    EEPROM.get(EEPROM_ADDR_MODE_NASOS_HEAT, modeNasosHeat);
    EEPROM.get(EEPROM_ADDR_HYSTERESIS, hysteresis);

    Serial.println("Loaded from EEPROM:");
    Serial.println("Mode Set Heat: " + String(modeSetHeat));
    Serial.println("Temperature Setpoint: " + String(tempSetpoint));
    Serial.println("Mode Nasos Boy: " + String(modeNasosBoy));
    Serial.println("Mode Nasos Heat: " + String(modeNasosHeat));
    Serial.println("Hysteresis: " + String(hysteresis));

    // Ініціалізація сенсора, реле, Wi-Fi та MQTT
    tempSensor.begin();
    relay.begin();
    wifi.connect();
    mqtt.begin();
    mqtt.connect();
    mqtt.subscribe("home/set/#");
    mqtt.setCallback(mqttCallback);

    // Ініціалізація OTA
    ArduinoOTA.setHostname("ESP_termostat");
    ArduinoOTA.begin();
}

void loop() {
    ArduinoOTA.handle();
    wifi.reconnectIfNeeded();
    mqtt.reconnectIfNeeded();

    // Оновлення логіки
    if (tempSensor.isTimeToMeasure()) {
        logic.update(modeSetHeat, modeNasosHeat, modeNasosBoy, tempSetpoint);
    }

    // Публікація температури кожну секунду
    static unsigned long lastTempPublishTime = 0;
    if (millis() - lastTempPublishTime >= 2000) {
        lastTempPublishTime = millis();
        float currentTemp = tempSensor.getTemperature();
        mqtt.publish(mqtt.getTempInTopic(), String(currentTemp).c_str());
        Serial.println("Published current temperature: " + String(currentTemp));
    }

    mqtt.loop();
}
