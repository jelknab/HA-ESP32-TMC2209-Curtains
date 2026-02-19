#include <Arduino.h>
#include "esp_system.h"
#include <TMCStepper.h>
#include <AccelStepper.h>
#include "OneButton.h"
#include "WiFi.h"
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include "HA_discovery_config.h"


#define MAX_SPEED (180 * steps_per_mm)
#define STALL_VALUE 50 // [0..255]
#define EN_DIR_STEP_OUTPUT 1
#define MQTT_MAX_PACKET_SIZE 2048

#define R_SENSE 0.11f

#define EN_PIN 2    // enable (CFG6)
#define DIR_PIN 18  // direction
#define STEP_PIN 23 // step

#define CLK_PIN 19
#define SPREAD_PIN 4

#define SW_RX 26 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX 27 // TMC2208/TMC2224 SoftwareSerial transmit pin

#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define BTN1 36 // NEXT
#define BTN2 34 // ENTER
#define BTN3 35 // MENU

#define SPI_MT_CS 15 // MT6816
#define SPI_CLK 14
#define SPI_MISO 12
#define SPI_MOSI 13

#define IIC_SCL 21
#define IIC_SDA 22

#define iStep 25
#define iDIR 32
#define iEN 33

HardwareSerial SerialDriver(Serial1);
TMC2209Stepper driver(&SerialDriver, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper(stepper.DRIVER, STEP_PIN, DIR_PIN);
OneButton button1(BTN1, true);
OneButton button2(BTN2, true);
OneButton button3(BTN3, true);
SPIClass MT6816;

constexpr uint32_t steps_per_mm = 80;

static double Monitor_Speed;

// int8_t Direction = 1;
double _lastLocation = 0;
double _currentLocation = 0;

long minimumPosition = 0;
long maximumPosition = 0;

// Timing
unsigned long lastMQTTReconnect = 0;
unsigned long lastWiFiReconnect = 0;

char deviceID[18];

void motor_init(void);
void MT6816_init(void);
int MT6816_read(void);
void Position1();
void Position2();
void Stop();
void Task1(void *pvParameters);
void Task2(void *pvParameters);

IPAddress server(192, 168, 1, 9);
WiFiClient espClient;

void MqttCallback(char *topic, byte *payload, unsigned int length)
{
    if (strcmp(topic, absolutePositionCommandTopic) == 0) {
        char msg[length + 1];

        memcpy(msg, payload, length);
        msg[length] = '\0';

        long targetPosition = atol(msg);   // convert to long

        stepper.enableOutputs();
        stepper.moveTo(targetPosition);
    }
    // Minumum button press 
    else if (strcmp(topic, minimumButtonCommandTopic) == 0) {
        minimumPosition = stepper.currentPosition();
    }
    // Maximum button press
    else if (strcmp(topic, maximumButtonCommandTopic) == 0) {
        maximumPosition = stepper.currentPosition();
    }
    // Relative position 0 - 100
    else if (strcmp(topic, coverCommandTopic) == 0) {
        char msg[length + 1];

        memcpy(msg, payload, length);
        msg[length] = '\0';

        long percentOpen = atol(msg);

        long target = minimumPosition + ((maximumPosition - minimumPosition) * percentOpen) / 100L;
        stepper.moveTo(target);
    }
    // Speed set
    else if (strcmp(topic, speedCommandTopic) == 0) {
        char msg[length + 1];

        memcpy(msg, payload, length);
        msg[length] = '\0';

        float speed = atof(msg);
        stepper.setSpeed(speed);
    }
}

PubSubClient pubSubClient(server, 1883, MqttCallback, espClient);

TimerHandle_t sensorTimer;
void publishState() {
    // Build the topic dynamically
    char topic[64];
    snprintf(topic, sizeof(topic), "homeassistant/curtains/%s/state", deviceID);

    // Build JSON payload
    JsonDocument doc;
    doc["position"] = stepper.currentPosition();
    doc["stallguard"] = driver.SG_RESULT();

    char payload[128];
    serializeJson(doc, payload, sizeof(payload));

    // Publish
    pubSubClient.publish(topic, payload);
}

void publishRelPosition() {
    long current = stepper.currentPosition();

    // Compute relative position as 0-100% within min-max range
    long percentage = 0;

    if (maximumPosition != minimumPosition) { // avoid division by zero
        percentage = ((current - minimumPosition) * 100L) / (maximumPosition - minimumPosition);

        // Clamp between 0-100
        if (percentage < 0) percentage = 0;
        if (percentage > 100) percentage = 100;
    }

    char payload[16];
    snprintf(payload, sizeof(payload), "%ld", percentage);

    pubSubClient.publish(coverPositionTopic, payload);
}

void PublishSensors(TimerHandle_t xTimer) 
{
    if (!pubSubClient.connected()) {
        return;
    }
    
    publishState();
    publishRelPosition();

    xTimerChangePeriod(sensorTimer, pdMS_TO_TICKS( stepper.isRunning() ? 100 : 1000 ), 10);
}

void setup()
{
    uint8_t baseMac[6];
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(deviceID, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);

    motor_init();
    xTaskCreatePinnedToCore(Task1, "Task1", 1024 * 10, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(Task2, "Task2", 1024 * 20, NULL, 2, NULL, 0);
    sensorTimer = xTimerCreate(
        "PublishSensors",                   // Timer name
        pdMS_TO_TICKS( 1000 ),      // 1s period
        pdTRUE,                         // Auto-reload (periodic timer)
        NULL,                           // Timer ID
        PublishSensors                   // Callback function
    );

    xTimerStart(sensorTimer, pdMS_TO_TICKS( 5000 ));
}

void handleWiFiReconnect() {
    if (WiFi.status() != WL_CONNECTED) {
        unsigned long now = millis();
        if (now - lastWiFiReconnect > 5000) { // every 5s
            lastWiFiReconnect = now;
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
    }
}

void handleMQTTReconnect() {
    if (!pubSubClient.connected() && WiFi.status() == WL_CONNECTED) {
        unsigned long now = millis();
        if (now - lastMQTTReconnect > 5000) { // every 5s
            lastMQTTReconnect = now;

            if (pubSubClient.connect(DEVICE_ID, MQTT_USER, MQTT_PASS)) {
                // Resubscribe topics
                pubSubClient.subscribe(absolutePositionCommandTopic);
                pubSubClient.subscribe(speedCommandTopic);
                pubSubClient.subscribe(minimumButtonCommandTopic);
                pubSubClient.subscribe(maximumButtonCommandTopic);
                pubSubClient.subscribe(coverCommandTopic);
            }
        }
    }
}

void loop()
{
    ArduinoOTA.handle();
    stepper.run();
    
    if (pubSubClient.connected()) {
        pubSubClient.loop();
    }

    handleWiFiReconnect();
    handleMQTTReconnect();
}

void Task1(void *pvParameters)
{
    MT6816_init();

    button1.attachClick(Position1);
    button2.attachClick(Position2);
    button3.attachClick(Stop);

    while (1)
    {
        button1.tick();
        button2.tick();
        button3.tick();
        delay(100);
    }
}

void Task2(void *pvParameters)
{
    bool isConnected = false;
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(250);
    }

    ArduinoOTA.begin();

    pubSubClient.setBufferSize(MQTT_MAX_PACKET_SIZE);
    if (pubSubClient.connect(DEVICE_ID, MQTT_USER, MQTT_PASS))
    {
        char topic[64];
        snprintf(topic, sizeof(topic), "homeassistant/device/curtain_%s/config", deviceID);

        char payload[MQTT_MAX_PACKET_SIZE];
        GetDeviceDiscoveryPayload(deviceID, payload, sizeof(payload));

        pubSubClient.publish(topic, payload);
        pubSubClient.subscribe(absolutePositionCommandTopic);
        pubSubClient.subscribe(minimumButtonCommandTopic);
        pubSubClient.subscribe(maximumButtonCommandTopic);
        pubSubClient.subscribe(coverCommandTopic);
    }

    vTaskDelete(NULL);
}

void MT6816_init(void)
{
    MT6816.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_MT_CS);
    pinMode(SPI_MT_CS, OUTPUT);
    MT6816.setClockDivider(SPI_CLOCK_DIV4);
    _lastLocation = (double)MT6816_read();
}

int MT6816_read(void)
{
    uint16_t temp[2];
    digitalWrite(SPI_MT_CS, LOW);
    MT6816.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE3));
    temp[0] = MT6816.transfer16(0x8300) & 0xFF;
    MT6816.endTransaction();
    digitalWrite(SPI_MT_CS, HIGH);

    digitalWrite(SPI_MT_CS, LOW);
    MT6816.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE3));
    temp[1] = MT6816.transfer16(0x8400) & 0xFF;
    MT6816.endTransaction();
    digitalWrite(SPI_MT_CS, HIGH);
    return (int)(temp[0] << 6 | temp[1] >> 2);
}

void motor_init(void)
{
    pinMode(CLK_PIN, OUTPUT);
    pinMode(SPREAD_PIN, OUTPUT);
    digitalWrite(CLK_PIN, LOW);
    digitalWrite(SPREAD_PIN, HIGH);

    pinMode(EN_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW); // Enable driver in hardware

    SerialDriver.begin(115200, SERIAL_8N1, SW_RX, SW_TX);
    driver.begin(); //  SPI: Init CS pins and possible SW SPI pins
    // UART: Init SW UART (if selected) with default 115200 baudrate
    driver.push();
    driver.pdn_disable(true);
    driver.mstep_reg_select(1);
    driver.blank_time(24);
    driver.toff(5);           // Enables driver in software
    driver.rms_current(1600); // Set motor RMS current
    driver.microsteps(8);     // Set microsteps to 1/16th
    driver.ihold(0);
    driver.iholddelay(10);
    driver.freewheel(1);
    driver.TPOWERDOWN(100);

    driver.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224
    driver.pwm_autoscale(true);   // Needed for stealthChop

    // stalling
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.semin(5);
    driver.semax(2);
    driver.sedn(0b01);
    driver.SGTHRS(STALL_VALUE);

    // driver->TSTEP();
    stepper.setMaxSpeed(MAX_SPEED);              // 100mm/s @ 80 steps/mm
    stepper.setAcceleration(500); // 2000mm/s^2
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(false, false, true);
}

void Position1()
{
    stepper.enableOutputs();
    stepper.moveTo(stepper.currentPosition() + 200);
}

void Position2()
{
    stepper.enableOutputs();
    stepper.moveTo(stepper.currentPosition() - 200);
}

void Stop()
{
    stepper.stop();
    while (stepper.isRunning())
    {
        stepper.run();
    }
    stepper.disableOutputs(); // ⭐ freespin
}

void PerformHoming()
{
}
