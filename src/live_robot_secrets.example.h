#pragma once

// Copy this file to live_robot_secrets.h. That filename is gitignored.
// Wi-Fi is station-only: firmware joins an existing WPA2 network and never
// creates an open access point.
#define DOMINO_LIVE_WIFI_ENABLED 1
#define DOMINO_LIVE_WIFI_SSID "your-wpa2-network"
#define DOMINO_LIVE_WIFI_PASSWORD "replace-with-a-long-password"
#define DOMINO_LIVE_WIFI_HOSTNAME "domino-robot"
#define DOMINO_LIVE_WIFI_PORT 8766

// Required for either wireless transport. Use the same value in the PC's
// DOMINO_ROBOT_LINK_KEY environment variable. Keep it at least 16 characters.
#define DOMINO_LIVE_LINK_KEY "replace-with-a-separate-long-random-key"

// Classic Bluetooth SPP appears as a serial port on the PC. Choose a unique
// device name and a non-default PIN of 4-16 digits.
#define DOMINO_LIVE_BLUETOOTH_ENABLED 1
#define DOMINO_LIVE_BLUETOOTH_NAME "Domino-LIVE"
#define DOMINO_LIVE_BLUETOOTH_PIN "739184"
