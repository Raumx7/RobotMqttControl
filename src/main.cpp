#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "RobotController.h"

// --- WiFi ---
constexpr auto SSID = "TecLab";
constexpr auto PASS = "Teclab2026";
constexpr unsigned CONNECTION_TIME_OUT = 10'000;

// --- MQTT ---
constexpr auto BROKER = "10.251.53.14";
constexpr uint16_t PORT = 1883;
constexpr auto SUB_TOPIC = "robot/command";
constexpr auto PUB_TOPIC_RESP = "robot/response";
constexpr auto PUB_TOPIC_STAT = "robot/status";

WiFiClient espClient;
PubSubClient client(espClient);

// --- Motors ---
constexpr uint8_t IN3 = 33;
constexpr uint8_t IN4 = 32;
constexpr uint8_t IN1 = 26;
constexpr uint8_t IN2 = 27;
constexpr uint8_t ENA1 = 14;
constexpr uint8_t ENA2 = 25;

L298N motors_l(IN3, IN4, ENA2);
L298N motors_r(IN1, IN2, ENA1);

RobotController robot(motors_l, motors_r);

void send_json_status() {
    static RobotParams last_params{};
    static unsigned long last_send = 0;
    static bool has_published = false;

    const RobotParams current = robot.get_params();

    const bool changed = !has_published || !robot.has_same_params(last_params);

    if (!changed) {
        return;
    }

    if (has_published && millis() - last_send < 500) {
        return;
    }

    char buffer[128];

    snprintf(buffer, sizeof(buffer),
        R"({"left":%d,"right":%d,"mode":"%s","brake_mode":"%s"})",
        current.l_power,
        current.r_power,
        fmt::to_str(current.auto_state),
        fmt::to_str(current.brake_mode)
    );

    if (client.publish(PUB_TOPIC_STAT, buffer)) {
        last_params = current;
        last_send = millis();
        has_published = true;
    }
}

void send_json_response(const ControllerResponse& response) {
    char buffer[64];

    if (response) {
        snprintf(buffer, sizeof(buffer),
            R"({"success":true,"command":"%s"})",
            fmt::to_str(*response)
            );
    }

    else {
        snprintf(buffer, sizeof(buffer),
            R"({"success":false,"error":"%s"})",
            fmt::to_str(response.error())
            );
    }
    client.publish(PUB_TOPIC_RESP, buffer);
}

void callback(char* /*topic*/, byte* payload, unsigned int length) {

    std::string_view cmd(reinterpret_cast<const char*>(payload), length);

    ControllerResponse response = robot.execute_command(cmd);   // returns command type or an error

    send_json_response(response);
}

void setup_wifi() {
    WiFi.begin(SSID, PASS);
    const unsigned start = millis();

    while (WiFi.status() != WL_CONNECTED && (millis() - start) <= CONNECTION_TIME_OUT) {
        Serial.print(".");
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nNo se pudo conectar al WiFi, reiniciando...");
        ESP.restart();
    }

    Serial.println("\nWiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
}

void reconnect() {
    robot.stop();

    while (!client.connected()) {
        String clientID = "ESP32-" + WiFi.macAddress();

        if (client.connect(clientID.c_str())) {
            Serial.println("Connected to MQTT broker");
            client.subscribe(SUB_TOPIC);
        } else {
            Serial.println("Connection to broker failed, trying again in 5 sec...");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);

    motors_l.init();
    motors_r.init();
    motors_r.set_reversed();
    robot.set_brake_mode(BrakeMode::Coast);

    setup_wifi();

    client.setServer(BROKER, PORT);
    client.setCallback(callback);
}

void loop() {
    send_json_status(); // sends robot status

    if (!client.connected()) {
        reconnect();
    }
    client.loop();  // check broker connection
    robot.loop(); // updates autonomous routine
}
