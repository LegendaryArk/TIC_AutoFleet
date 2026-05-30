#include <WiFi.h>
#include "env.h"

// ===== UART to PRIZM (ESP32-C6) =====
HardwareSerial PRIZM(1);
#define RXD1 4
#define TXD1 5

// ===== TCP Server =====
WiFiServer server(81);
WiFiClient client;

bool clientConnected = false;

// ===== Setup =====
void setup()
{
    Serial.begin(115200);
    delay(500);

    PRIZM.begin(38400, SERIAL_8N1, RXD1, TXD1);

    WiFi.begin(ssid, password);
    WiFi.setTxPower(WIFI_POWER_13dBm);
    Serial.print("[ESP32] Connecting to WiFi");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\n[ESP32] WiFi connected");
    Serial.print("[ESP32] IP: ");
    Serial.println(WiFi.localIP());

    server.begin();
    Serial.println("[ESP32] TCP server started on port 81");
    Serial.println("[ESP32] Waiting for client...");
}

// ===== Main Loop =====
void loop()
{
    if (!client || !client.connected())
    {
        WiFiClient newClient = server.accept();
        if (newClient)
        {
            client = newClient;
            clientConnected = true;
            Serial.println("[ESP32] Client connected");
            client.println("{\"type\":\"esp32_ready\"}");
        }
    }

    // TCP → PRIZM
    if (client && client.connected() && client.available())
    {
        while (client.available())
        {
            char c = client.read();
            PRIZM.write(c);
            Serial.write(c);
        }
    }

    // PRIZM → TCP
    static String prizmBuffer = "";

    while (PRIZM.available())
    {
        char c = PRIZM.read();
        prizmBuffer += c;

        if (c == '\n')
        {
            prizmBuffer.trim();
            if (prizmBuffer.length() > 0)
            {
                Serial.print("[PRIZM → TCP] ");
                Serial.println(prizmBuffer);
                if (client && client.connected())
                    client.println(prizmBuffer);
            }
            prizmBuffer = "";
        }
    }

    delay(2);
}
