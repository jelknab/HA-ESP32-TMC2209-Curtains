#include <stddef.h>
#include <ArduinoJson.h>

char coverPositionTopic[64];
char coverCommandTopic[64];
char absolutePositionCommandTopic[64];
char minimumButtonCommandTopic[64];
char maximumButtonCommandTopic[64];

bool GetDeviceDiscoveryPayload(const char* id, char* buffer, size_t bufferSize) {
    JsonDocument doc;

    // ---- dev ----
    JsonObject dev = doc["dev"].to<JsonObject>();
    dev["ids"] = id;
    dev["name"] = "Curtain actuator";
    dev["mf"] = "Jellie productions";
    dev["mdl"] = "V1";
    dev["sw"] = "1.2";
    dev["sn"] = id;
    dev["hw"] = "ESP PICO D4 - TMC2209";

    // ---- o ----
    JsonObject o = doc["o"].to<JsonObject>();
    o["name"] = "Jellie productions";
    o["sw"] = "1.2";
    o["url"] = "http://localhost/support";

    // ---- cmps ----
    JsonObject cmps = doc["cmps"].to<JsonObject>();

    JsonObject positionCmp = cmps["position"].to<JsonObject>();
    positionCmp["p"] = "sensor";
    positionCmp["name"] = "position";
    // positionCmp["device_class"] = "Number";
    positionCmp["unit_of_measurement"] = "Steps";
    positionCmp["value_template"] = "{{ value_json.position }}";

    char positionId[32];
    snprintf(positionId, sizeof(positionId), "%s_position", id);
    positionCmp["unique_id"] = positionId;

    

    JsonObject cover = cmps["cover"].to<JsonObject>();
    cover["p"] = "cover";
    cover["name"] = "curtain";

    snprintf(coverPositionTopic, sizeof(coverPositionTopic), "homeassistant/cover/%s/position", id);
    cover["position_topic"] = coverPositionTopic;

    snprintf(coverCommandTopic, sizeof(coverCommandTopic), "homeassistant/cover/%s/position_set", id);
    cover["set_position_topic"] = coverCommandTopic;
    cover["position_open"] = 100;
    cover["position_closed"] = 0;

    char coverId[32];
    snprintf(coverId, sizeof(coverId), "%s_cover", id);
    cover["unique_id"] = coverId;



    JsonObject absolutePosition = cmps["absolutePosition"].to<JsonObject>();
    absolutePosition["p"] = "number";
    absolutePosition["name"] = "position";
    absolutePosition["mode"] = "box";
    absolutePosition["min"] = -100000;
    absolutePosition["max"] = 100000;

    snprintf(absolutePositionCommandTopic, sizeof(absolutePositionCommandTopic), "homeassistant/cover/%s/absolute_position_set", id);
    absolutePosition["command_topic"] = absolutePositionCommandTopic;
    
    char absolutePositionId[32];
    snprintf(absolutePositionId, sizeof(absolutePositionId), "%s_position", id);
    absolutePosition["unique_id"] = absolutePositionId;



    JsonObject minimumButton = cmps["minimumButton"].to<JsonObject>();
    minimumButton["p"] = "button";
    minimumButton["name"] = "set minimum";

    snprintf(minimumButtonCommandTopic, sizeof(minimumButtonCommandTopic), "homeassistant/cover/%s/minimum_position_set", id);
    minimumButton["command_topic"] = minimumButtonCommandTopic;

    char minimumButtonId[32];
    snprintf(minimumButtonId, sizeof(minimumButtonId), "%s_minimum_button", id);
    minimumButton["unique_id"] = minimumButtonId;



    JsonObject maximumButton = cmps["maximumButton"].to<JsonObject>();
    maximumButton["p"] = "button";
    maximumButton["name"] = "set maximum";

    snprintf(maximumButtonCommandTopic, sizeof(maximumButtonCommandTopic), "homeassistant/cover/%s/maximum_position_set", id);
    maximumButton["command_topic"] = maximumButtonCommandTopic;

    char maximumButtonId[32];
    snprintf(maximumButtonId, sizeof(maximumButtonId), "%s_maximum_button", id);
    maximumButton["unique_id"] = maximumButtonId;



    // ---- root values ----
    char stateTopic[64];
    snprintf(stateTopic, sizeof(stateTopic), "homeassistant/curtains/%s/state", id);

    doc["state_topic"] = stateTopic;
    doc["qos"] = 2;

    // Serialize safely into provided buffer
    size_t written = serializeJson(doc, buffer, bufferSize);

    return (written > 0 && written < bufferSize);
}