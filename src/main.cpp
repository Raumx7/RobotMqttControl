#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <SimpleOLED.h>

#include "RobotController.h"
#include "AHT10_Light.h"
#include "format.h"

// --- WiFi ---
constexpr auto SSID = "TecLab";
constexpr auto PASS = "Teclab2026";
constexpr unsigned long CONNECTION_TIME_OUT = 10'000;

// --- MQTT ---
constexpr auto BROKER = "10.121.119.14";
constexpr uint16_t PORT = 1883;
constexpr auto SUB_TOPIC = "robot/command";
constexpr auto PUB_TOPIC_RESP = "robot/response";
constexpr auto PUB_TOPIC_SENS = "robot/telemetry";
constexpr auto PUB_TOPIC_STAT = "robot/status";

// --- Clients ---
WiFiClient espClient;
PubSubClient client(espClient);

// --- Motors ---
constexpr uint8_t IN1 = 12;
constexpr uint8_t IN2 = 13;
constexpr uint8_t IN3 = 25;
constexpr uint8_t IN4 = 26;

constexpr uint8_t ENA1 = 14;
constexpr uint8_t ENA2 = 27;

L298N motors_l(IN1, IN2, ENA1);
L298N motors_r(IN3, IN4, ENA2);

// --- Robot cmd handler ---
RobotController robot(motors_l, motors_r);

// --- OLED ---
SimpleOLED oled;

// --- Sensors ---
AHT10_Light aht;

// --- Sensor Data ---
SensorData sensor_data{};

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

void send_json_telemetry() {
    static unsigned long last_send = 0;
    static bool has_published = false;

    if (has_published && millis() - last_send < 1000) {
        return;
    }

    char buffer[128];

    snprintf(buffer, sizeof(buffer),
    R"({"sensor_id":"aht-01","temp":%.1f,"temp_unit":"C","hum":%.1f,"hum_unit":"%%"})",
    sensor_data.temperature,
    sensor_data.humidity
    );
    
    if (client.publish(PUB_TOPIC_SENS, buffer)) {
        last_send = millis();
        has_published = true;
    }
}

void send_json_response(const ControllerResponse& response) {
    char buffer[64];

    if (response) {
        snprintf(buffer, sizeof(buffer),
            R"({"success":true,"command":"%s"})",
            fmt::to_str(response.value())
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

void display_wifi_info() {
    oled.clear();
    oled.setCursor(0, 0);

    oled.println(text::line);
    oled.println(text::wifi);

    char buffer[32];

    oled.setCursor(COL[0], ROW[4]);
    snprintf(buffer, sizeof(buffer), "WIFI: %s", fmt::to_str(WiFi.status()));
    oled.println(buffer);

    snprintf(buffer, sizeof(buffer), "MQTT: %s", fmt::to_str(client.state()));
    oled.print(buffer);

    oled.display();
}

void display_sensor_data() {
    static unsigned long last_sensor_read = 0;
    static unsigned long last_oled_draw = 0;
    const unsigned long now = millis();

    // Read sensor every 1000 ms
    if (now - last_sensor_read >= 1000) {
        aht.read_data(sensor_data.temperature, sensor_data.humidity);
        last_sensor_read = now;
    }

    // Refresh OLED screen every 250 ms
    if (now - last_oled_draw >= 250) {
        last_oled_draw = now;

        oled.clear();
        oled.setCursor(0, 0);

        oled.println(text::line);
        oled.println(text::data);

        char buffer[32];

        oled.setCursor(COL[0], ROW[4]);
        snprintf(buffer, sizeof(buffer), "Temperature: %.1f C", sensor_data.temperature);
        oled.println(buffer);

        snprintf(buffer, sizeof(buffer), "Humidity: %.1f %%", sensor_data.humidity);
        oled.print(buffer);

        oled.display();
    }
}

void update_data() {
    static unsigned long last_mode_switch = 0;
    static bool showing_sensor = false;

    const unsigned long now = millis();

    if (!showing_sensor) {
        if (now - last_mode_switch >= 2000) {
            last_mode_switch = now;
            showing_sensor = true;
        }
    } else {
        display_sensor_data();  // Shows and updates sensor data on the OLED (5s)

        if (now - last_mode_switch >= 5000) {
            last_mode_switch = now;
            showing_sensor = false;
            display_wifi_info(); // Draws the Wi-Fi screen only once (2s)
        }
    }
}

void setup_wifi() {
    WiFi.begin(SSID, PASS);
    const unsigned start = millis();

    oled.clear();
    oled.println("WIFI: connecting...");
    oled.display();

    while (WiFi.status() != WL_CONNECTED && (millis() - start) <= CONNECTION_TIME_OUT) {
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        oled.println("WIFI: disconnected!");
        oled.print("Rebooting soon...");
        oled.display();
        delay(2000);
        ESP.restart();
    }

    oled.println("WIFI: connected!");
    oled.print("IP: ");
    oled.print(WiFi.localIP().toString().c_str());
    oled.display();
    delay(2000);
}

void reconnect() {
    robot.stop();

    oled.clear();
    oled.setCursor(0, 0);

    bool displayed = false;

    while (!client.connected()) {
        String clientID = "ESP32-" + WiFi.macAddress();

        if (client.connect(clientID.c_str())) {
            oled.print("MQTT: connected");
            oled.display();
            delay(1000);
            client.subscribe(SUB_TOPIC);
        } else {
            if (!displayed) {
                oled.print("MQTT: disconnected...");
                oled.display();
                displayed = true;
            }
            delay(5000);
        }
    }
}

void setup() {
    oled.begin();
    aht.begin();

    motors_l.begin();
    motors_r.begin();
    motors_r.reversed();

    robot.set_brake_mode(BrakeMode::Coast);

    setup_wifi();
    espClient.setTimeout(100);

    client.setServer(BROKER, PORT);
    client.setCallback(callback);
}

void loop() {
    update_data();  // updates oled info and sensor data
    send_json_status(); // sends robot status
    send_json_telemetry(); // sends sensor data

    if (!client.connected()) {
        reconnect();
    }
    client.loop();  // check broker connection
    robot.loop(); // updates autonomous routine
}
