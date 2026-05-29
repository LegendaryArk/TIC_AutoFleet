//Copy this into an arduino sketch OR open in Platform io 
//ensure the wifi credentials are in the same folder

#include <WiFi.h>
#include "env.h"

//For the ESP32 Dev Module or ESP32C6 Dev Module

// ===== UART to PRIZM =====
HardwareSerial PRIZM(2);
#define RXD2 4
#define TXD2 13

//Uncomment for ESP32C6 dev module (the esp32 with a usbc)
// HardwareSerial PRIZM(1);
// #define RXD1 4
// #define TXD1 5

// ===== TCP Server =====
WiFiServer server(81);
WiFiClient client;

bool clientConnected = false;

// ===== Setup =====
void setup()
{
    // INTERNAL SERIAL FOR DEBUGGING
    Serial.begin(115200);
    delay(500);

    // PRIZM MUST LOOK AT THIS EXACT BAUDRATE AND CONFIGURATION
    PRIZM.begin(38400, SERIAL_8N1, RXD2, TXD2);

    // Connect WiFi
    WiFi.begin(ssid, password);
    WiFi.setTxPower(WIFI_POWER_13dBm); // Set WiFi transmit power to 13dBm (default is 19.5dBm)
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
    // Accept new client if needed
    if (!client || !client.connected())
    {
        WiFiClient newClient = server.available();
        if (newClient)
        {
            client = newClient;
            clientConnected = true;

            Serial.println("[ESP32] Client connected");

            // Send ready message
            client.println("{\"type\":\"esp32_ready\"}");
        }
    }

    // ===== TCP → PRIZM =====
    if (client && client.connected() && client.available())
    {
        while (client.available())
        {
            char c = client.read();
            PRIZM.write(c);

            // Debug
            Serial.write(c);
        }
    }

    // ===== PRIZM → TCP =====
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
                {
                    client.println(prizmBuffer);
                }
            }

            prizmBuffer = "";
        }
    }

    // Small yield for stability
    delay(2);
}
