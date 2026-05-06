#pragma once
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "../config.h"
#include "../mapping/occupancy_grid.h"

// ─────────────────────────────────────────────
//  WebServer
//  Core 0 task: serves web UI from LittleFS
//  and streams robot state/map via WebSocket
// ─────────────────────────────────────────────
class WebServer {
public:
    WebServer(uint16_t port = 80);

    // Call once from Core 0 setup
    void begin(const char* ssid, const char* password);

    // Call periodically from Core 0 loop (20Hz)
    // Broadcasts state JSON + map delta when dirty
    void broadcastState(const RobotState& state,
                        const OccupancyGrid& grid,
                        SemaphoreHandle_t mutex);

    // Latest command received from WebSocket (polled by Core 1)
    bool popCommand(RobotCommand& out);

private:
    AsyncWebServer   _server;
    AsyncWebSocket   _ws;
    QueueHandle_t    _cmd_queue;

    void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                   AwsEventType type, void* arg, uint8_t* data, size_t len);

    void handleMessage(const char* msg);

    // JSON builders
    String buildStateJson(const RobotState& state);
    String buildMapJson(const OccupancyGrid& grid);
};
