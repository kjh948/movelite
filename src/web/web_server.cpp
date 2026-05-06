#include "web_server.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h> // Ensure you add bblanchon/ArduinoJson to lib_deps

WebServer::WebServer(uint16_t port)
    : _server(port), _ws("/ws") {
    _cmd_queue = xQueueCreate(10, sizeof(RobotCommand));
}

void WebServer::begin(const char* ssid, const char* password) {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected. IP: ");
    Serial.println(WiFi.localIP());

    // Serve static files from LittleFS
    _server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    // WebSocket event handler
    _ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                       AwsEventType type, void* arg, uint8_t* data, size_t len) {
        onWsEvent(server, client, type, arg, data, len);
    });
    _server.addHandler(&_ws);

    _server.begin();
}

void WebServer::onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                          AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WS Client %u connected\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WS Client %u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0; // null terminate
            handleMessage((const char*)data);
        }
    }
}

void WebServer::handleMessage(const char* msg) {
    // Parse JSON
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, msg);
    if (err) return;

    const char* type = doc["type"];
    if (!type) return;

    RobotCommand cmd;
    if (strcmp(type, "joystick") == 0) {
        cmd.type = CmdType::JOYSTICK;
        cmd.param1 = doc["v"] | 0.0f;
        cmd.param2 = doc["omega"] | 0.0f;
    } else if (strcmp(type, "goal") == 0) {
        cmd.type = CmdType::SET_GOAL;
        cmd.param1 = doc["gx"] | 0.0f;
        cmd.param2 = doc["gy"] | 0.0f;
    } else if (strcmp(type, "cmd") == 0) {
        const char* action = doc["action"];
        if (action && strcmp(action, "start_mapping") == 0) cmd.type = CmdType::START_MAPPING;
        else if (action && strcmp(action, "stop_mapping") == 0) cmd.type = CmdType::STOP_MAPPING;
        else if (action && strcmp(action, "reset") == 0) cmd.type = CmdType::RESET;
    }
    
    if (cmd.type != CmdType::NONE) {
        xQueueSend(_cmd_queue, &cmd, 0);
    }
}

bool WebServer::popCommand(RobotCommand& out) {
    return xQueueReceive(_cmd_queue, &out, 0) == pdTRUE;
}

void WebServer::broadcastState(const RobotState& state,
                               const OccupancyGrid& grid,
                               SemaphoreHandle_t mutex) {
    if (_ws.count() == 0) return;

    // Send state JSON
    String stateJson = buildStateJson(state);
    _ws.textAll(stateJson);

    // Send map if dirty (in a real app, send chunks or diffs if large)
    if (state.map_dirty) {
        String mapJson = buildMapJson(grid);
        _ws.textAll(mapJson);
    }
}

String WebServer::buildStateJson(const RobotState& state) {
    StaticJsonDocument<256> doc;
    doc["type"] = "state";
    doc["x"] = state.x;
    doc["y"] = state.y;
    doc["theta"] = state.theta;
    doc["mode"] = static_cast<int>(state.mode);
    String out;
    serializeJson(doc, out);
    return out;
}

// For simplicity, we send the entire map as a base64 string or simple array.
// Base64 encoding the grid data is much more efficient for WebSockets.
#include <libb64/cencode.h>
String WebServer::buildMapJson(const OccupancyGrid& grid) {
    StaticJsonDocument<256> doc;
    doc["type"] = "map";
    doc["w"] = OccupancyGrid::W;
    doc["h"] = OccupancyGrid::H;
    
    base64_encodestate _state;
    base64_init_encodestate(&_state);
    
    // Allocate space for base64 string: 4/3 * size + padding
    int max_len = (OccupancyGrid::W * OccupancyGrid::H) * 4 / 3 + 10;
    char* b64 = (char*)malloc(max_len);
    if (!b64) return "{}";
    
    int c1 = base64_encode_block((const char*)grid.data(), OccupancyGrid::W * OccupancyGrid::H, b64, &_state);
    int c2 = base64_encode_blockend(b64 + c1, &_state);
    b64[c1 + c2] = 0; // null terminate, though encode_blockend usually adds \n

    // Remove \n if present
    for(int i=0; i<c1+c2; i++) {
        if(b64[i] == '\n') b64[i] = 0;
    }

    doc["data"] = b64;
    String out;
    serializeJson(doc, out);
    free(b64);
    return out;
}
