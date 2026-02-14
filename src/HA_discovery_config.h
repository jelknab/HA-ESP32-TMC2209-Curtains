#include <stddef.h>
#include <ArduinoJson.h>

bool GetDeviceDiscoveryPayload(const char* id, char* buffer, size_t bufferSize) {
    JsonDocument doc;

    // ---- dev ----
    JsonObject dev = doc["dev"].to<JsonObject>();
    dev["ids"] = id;
    dev["name"] = "Curtain actuator";
    dev["mf"] = "Jellie productions";
    dev["mdl"] = "V1";
    dev["sw"] = "1.0";
    dev["sn"] = id;
    dev["hw"] = "ESP PICO D4 - TMC2209";

    // ---- o ----
    JsonObject o = doc["o"].to<JsonObject>();
    o["name"] = "Jellie productions";
    o["sw"] = "1.0";
    o["url"] = "http://localhost/support";

    // ---- cmps ----
    JsonObject cmps = doc["cmps"].to<JsonObject>();

    JsonObject temp = cmps["position"].to<JsonObject>();
    temp["p"] = "sensor";
    // temp["device_class"] = "Number";
    temp["unit_of_measurement"] = "Steps";
    temp["value_template"] = "{{ value_json.position }}";

    char positionId[32];
    snprintf(positionId, sizeof(positionId), "%s_position", id);
    temp["unique_id"] = positionId;

    // ---- root values ----
    char stateTopic[64];
    snprintf(stateTopic, sizeof(stateTopic), "homeassistant/curtains/%s/state", id);

    doc["state_topic"] = stateTopic;
    doc["qos"] = 2;

    // Serialize safely into provided buffer
    size_t written = serializeJson(doc, buffer, bufferSize);

    return (written > 0 && written < bufferSize);
}