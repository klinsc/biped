#ifndef WEBHANDLER_H
#define WEBHANDLER_H

#include <ESPAsyncWebServer.h>

class WebHandler {
public:
    WebHandler();
    void begin();

private:
    AsyncWebServer server;
};

#endif
