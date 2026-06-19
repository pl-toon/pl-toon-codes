/*
 * MOBILE VERSION - Train Control Firmware with WiFi AP + WebSocket Dashboard
 * v2.1: PID Control, Step Response, Deadband Calibration
 *
 * Creates a WiFi Access Point (TrenCE-<trainID>) and serves a mobile web
 * dashboard at 192.168.4.1. Communication via WebSocket (JSON).
 *
 * EXPERIMENT MODES:
 * - PID Control:          Closed-loop distance tracking with Kp/Ki/Kd tuning
 * - Step Response:        Open-loop motor step with configurable amplitude/duration
 * - Deadband Calibration: Auto-detects minimum PWM to move the train
 *
 * CONFIGURATION COMMANDS (Serial, 115200 baud):
 * - SET_TRAIN:trainID:port   - Configure train ID (e.g., SET_TRAIN:trainA:5555)
 * - SET_MOTOR_INV:0|1        - Set motor direction inversion for this train
 * - GET_TRAIN                - Display current configuration
 * - RESET_TRAIN              - Clear configuration and restart
 * - STATUS                   - Show connection status
 *
 * LED FEEDBACK:
 * - Fast blink (200ms)  : Not configured, waiting for setup
 * - Slow blink (1s)     : Configured, starting AP
 * - Solid ON            : AP active, web server running
 * - 3 quick flashes     : Configuration saved successfully
 *
 * WebSocket Protocol (JSON):
 * - ESP→Phone data (PID):      {"t":"d","m":"pid","ts":1234,"d":15.2,"r":10.0,"e":-5.2,"kp":2.5,"ki":0.8,"kd":1.2,"u":127}
 * - ESP→Phone data (Step):     {"t":"d","m":"step","ts":1234,"d":15.2,"pwm":100,"amp":0.5,"dir":1}
 * - ESP→Phone data (Deadband): {"t":"d","m":"deadband","ts":1234,"d":15.2,"pwm":50,"d0":15.0,"motion":0}
 * - Phone→ESP cmd:             {"t":"cmd","c":"kp","v":2.5}
 * - Phone→ESP start:           {"t":"cmd","c":"start","m":"pid"}
 * - ESP→Phone ack:             {"t":"ack","c":"kp","v":2.5}
 * - ESP→Phone status:          {"t":"status","mode":"pid","active":true}
 * - ESP→Phone info:            {"t":"info","id":"trainA","ip":"192.168.4.1","ap":"TrenCE-trainA","fw":"2.3-mobile"}
 *
 * Dependencies (Arduino Library Manager):
 * - ESPAsyncWebServer by mathieucarbou (v3.0+)
 * - AsyncTCP by mathieucarbou
 * - ArduinoJson by Benoit Blanchon (v7.x)
 * - PID_v1_bc
 * - VL53L0X by Pololu
 * - LittleFS (built into ESP32 core)
 */

#include <PID_v1_bc.h>
#include <WiFi.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <DNSServer.h>

// =============================================================================
// Firmware Version
// =============================================================================
#define FW_VERSION "2.3-mobile"

// =============================================================================
// EEPROM Configuration Storage
// =============================================================================
Preferences preferences;

// Configuration variables (loaded from EEPROM)
String train_id = "";
int configured_udp_port = 5555;  // Kept for backward compat / serial config
bool is_configured = false;
bool motor_inverted = false;

// LED pin for status feedback
#define STATUS_LED 2

// Configuration mode variables
unsigned long last_led_toggle = 0;
int led_blink_interval = 200;
bool led_state = false;

// =============================================================================
// WiFi AP Configuration
// =============================================================================
String ap_ssid = "";           // Generated from train_id: "TrenCE-trainA"
const char* ap_password = "train123";  // AP password
IPAddress ap_ip(192, 168, 4, 1);
IPAddress ap_gateway(192, 168, 4, 1);
IPAddress ap_subnet(255, 255, 255, 0);

// =============================================================================
// Web Server + WebSocket
// =============================================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Captive portal: wildcard DNS so any hostname resolves to the AP, and joining
// the network opens the dashboard automatically (no IP to type).
DNSServer dnsServer;
const byte DNS_PORT = 53;

// WebSocket state
bool ws_client_connected = false;
uint32_t ws_packet_count = 0;

// =============================================================================
// Experiment Mode
// =============================================================================
#define PID_MODE 0
#define STEP_MODE 1
#define DEADBAND_MODE 2
int currentExperimentMode = PID_MODE;
bool experimentActive = false;

// =============================================================================
// Motor Configuration
// =============================================================================
const int STBY = 7;
const int Control_fwd = 8;
const int Control_back = 18;
const int Control_v = 17;
int MotorSpeed = 0;

int PIDMotorDirection = 1;
int StepMotorDirection = 1;
int DeadbandMotorDirection = 1;
int MotorDirection = 1;

// =============================================================================
// ToF Sensor
// =============================================================================
VL53L0X SensorToF;
double medi = 0;
double distancia = 25;
double old_d = 0;

// Moving average filter
int arrNumbers[3] = {0};
int pos = 0;
float newAvg = 0;
float oldAvg = 0;
int newSum = 0;
long sum = 0;
int len = sizeof(arrNumbers) / sizeof(int);

// =============================================================================
// PID Mode Variables
// =============================================================================
double x_ref = 10;
double error_distancia = 0;
double u_distancia = 0;
double rf = 0;
double Kp = 0, Ki = 0, Kd = 0;
int SampleTime = 50;
int umin = -1024, umax = 1024;
double etha = 0.5;
int deadband = 50;
int lim = 10;
double ponderado = 0;
bool flag_pid = true;
uint32_t tiempo_inicial_pid = 0;
bool pid_params_changed = false;

// Sensor warm-up for PID mode
const int PID_WARMUP_SAMPLES = 5;
int pidWarmupCounter = 0;

PID myPID(&error_distancia, &u_distancia, &rf, Kp, Ki, Kd, DIRECT);

// =============================================================================
// Step Response Mode Variables
// =============================================================================
double v_batt = 8.4;
double StepAmplitude = 0;
uint32_t StepTime = 0;
uint32_t StepTimeDuration = 0;
uint32_t delta = 0;
uint32_t tiempo_inicial_step = 0;
bool flag_step = true;
double u_step;
const int STEP_WARMUP_SAMPLES = 5;
const int STEP_BASELINE_SAMPLES = 3;
int stepWarmupCounter = 0;
int stepBaselineCounter = 0;
double appliedStepValue = 0.0;

// =============================================================================
// Deadband Calibration Mode Variables
// =============================================================================
bool flag_deadband = true;
uint32_t tiempo_inicial_deadband = 0;
int calibrated_deadband = 0;
double initial_distance = 0;
int pwm_increment = 1;
int pwm_delay = 40;
double motion_threshold = 0.08;
int max_pwm_test = 800;
bool motion_detected = false;
int consecutive_motion_count = 0;
const int MOTION_CONFIRM_COUNT = 2;

// =============================================================================
// Configuration Mode Functions
// =============================================================================

void loadConfiguration() {
  preferences.begin("train-config", false);
  is_configured = preferences.getBool("configured", false);
  motor_inverted = preferences.getBool("motor_inv", false);

  if (is_configured) {
    train_id = preferences.getString("train_id", "");
    configured_udp_port = preferences.getInt("udp_port", 5555);

    Serial.println("Configuration loaded from EEPROM:");
    Serial.println("  Train ID: " + train_id);
    Serial.println("  UDP Port: " + String(configured_udp_port));
    Serial.println("  Motor inverted: " + String(motor_inverted ? "YES" : "NO"));
  }

  preferences.end();
}

void saveConfiguration(String id, int port) {
  preferences.begin("train-config", false);
  preferences.putString("train_id", id);
  preferences.putInt("udp_port", port);
  preferences.putBool("configured", true);
  preferences.end();

  Serial.println("\nConfiguration saved to EEPROM!");
  Serial.println("  Train ID: " + id);
  Serial.println("  UDP Port: " + String(port));

  blinkLED(3, 150);

  Serial.println("\nRebooting...");
  delay(1000);
  ESP.restart();
}

void saveMotorInversion(bool inverted) {
  preferences.begin("train-config", false);
  preferences.putBool("motor_inv", inverted);
  preferences.end();

  motor_inverted = inverted;

  Serial.println("\nMotor direction configuration saved!");
  Serial.println("  Motor inverted: " + String(motor_inverted ? "YES" : "NO"));
  Serial.println("  Use Step mode Forward to verify physical direction.");
}

void resetConfiguration() {
  preferences.begin("train-config", false);
  preferences.clear();
  preferences.end();

  Serial.println("\nConfiguration cleared!");
  Serial.println("Rebooting to config mode...");
  delay(1000);
  ESP.restart();
}

void printConfiguration() {
  Serial.println("\n========================================");
  Serial.println("CURRENT CONFIGURATION");
  Serial.println("========================================");

  if (is_configured) {
    Serial.println("Status: CONFIGURED");
    Serial.println("Train ID: " + train_id);
    Serial.println("AP SSID: " + ap_ssid);
    Serial.println("AP IP: 192.168.4.1");
    Serial.println("Motor inverted: " + String(motor_inverted ? "YES" : "NO"));
    Serial.println("WebSocket clients: " + String(ws.count()));
    Serial.println("Experiment active: " + String(experimentActive ? "YES" : "NO"));
  } else {
    Serial.println("Status: NOT CONFIGURED");
    Serial.println("Motor inverted: " + String(motor_inverted ? "YES" : "NO"));
    Serial.println("Use: SET_TRAIN:trainID:port");
  }

  Serial.println("========================================\n");
}

void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(delayMs);
    digitalWrite(STATUS_LED, LOW);
    delay(delayMs);
  }
}

void checkSerialConfig() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("SET_TRAIN:")) {
      int firstColon = command.indexOf(':');
      int secondColon = command.indexOf(':', firstColon + 1);

      if (secondColon > 0) {
        String id = command.substring(firstColon + 1, secondColon);
        String portStr = command.substring(secondColon + 1);
        int port = portStr.toInt();

        if (id.length() > 0 && port > 0) {
          Serial.println("\nConfiguring train...");
          Serial.println("  Train ID: " + id);
          Serial.println("  UDP Port: " + String(port));
          saveConfiguration(id, port);
        } else {
          Serial.println("ERROR: Invalid train ID or port");
          Serial.println("Usage: SET_TRAIN:trainA:5555");
        }
      } else {
        Serial.println("ERROR: Invalid format");
        Serial.println("Usage: SET_TRAIN:trainA:5555");
      }
    }
    else if (command.startsWith("SET_MOTOR_INV:")) {
      String invStr = command.substring(command.indexOf(':') + 1);
      invStr.trim();

      if (invStr == "0" || invStr == "1") {
        saveMotorInversion(invStr == "1");
      } else {
        Serial.println("ERROR: Invalid motor inversion value");
        Serial.println("Usage: SET_MOTOR_INV:0  or  SET_MOTOR_INV:1");
      }
    }
    else if (command == "GET_TRAIN") {
      printConfiguration();
    }
    else if (command == "RESET_TRAIN") {
      Serial.println("Resetting configuration...");
      resetConfiguration();
    }
    else if (command == "STATUS") {
      printConfiguration();
    }
    else {
      Serial.println("Unknown command: " + command);
      Serial.println("Available commands:");
      Serial.println("  SET_TRAIN:trainID:port - Configure ESP32");
      Serial.println("  SET_MOTOR_INV:0|1      - Set motor direction inversion");
      Serial.println("  GET_TRAIN              - Show configuration");
      Serial.println("  RESET_TRAIN            - Clear configuration");
      Serial.println("  STATUS                 - Show status");
    }
  }
}

void enterConfigMode() {
  Serial.println("\n========================================");
  Serial.println("CONFIGURATION MODE (Mobile Firmware)");
  Serial.println("========================================");
  Serial.println("Commands:");
  Serial.println("  SET_TRAIN:trainA:5555  - Configure ESP32");
  Serial.println("  SET_MOTOR_INV:0|1      - Set motor direction inversion");
  Serial.println("  GET_TRAIN              - Show configuration");
  Serial.println("  RESET_TRAIN            - Clear configuration");
  Serial.println("========================================");
  Serial.println("Waiting for configuration...");
  Serial.println("(LED will blink fast until configured)\n");

  while (!is_configured) {
    if (millis() - last_led_toggle > led_blink_interval) {
      led_state = !led_state;
      digitalWrite(STATUS_LED, led_state ? HIGH : LOW);
      last_led_toggle = millis();
    }
    checkSerialConfig();
    delay(10);
  }
}

void updateStatusLED() {
  if (!is_configured) {
    // Fast blink - not configured
    if (millis() - last_led_toggle > 200) {
      led_state = !led_state;
      digitalWrite(STATUS_LED, led_state ? HIGH : LOW);
      last_led_toggle = millis();
    }
  }
  else if (!ws_client_connected) {
    // Slow blink - AP active but no client
    if (millis() - last_led_toggle > 1000) {
      led_state = !led_state;
      digitalWrite(STATUS_LED, led_state ? HIGH : LOW);
      last_led_toggle = millis();
    }
  }
  else {
    // Solid on - client connected
    digitalWrite(STATUS_LED, HIGH);
  }
}

// =============================================================================
// WiFi AP Setup
// =============================================================================

void setup_wifi_ap() {
  ap_ssid = "TrenCE-" + train_id;

  Serial.println("Starting WiFi Access Point...");
  Serial.println("  SSID: " + ap_ssid);
  Serial.println("  Password: " + String(ap_password));
  Serial.println("  IP: 192.168.4.1");

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
  WiFi.softAP(ap_ssid.c_str(), ap_password);

  // Disable power save for low latency
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Captive portal: route every DNS query to the AP address
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", ap_ip);

  Serial.println("AP started successfully!");
  Serial.println("  Connect phone to: " + ap_ssid);
  Serial.println("  Open browser: http://192.168.4.1");
}

// =============================================================================
// LittleFS Setup
// =============================================================================

bool setup_littlefs() {
  Serial.println("Mounting LittleFS...");

  if (!LittleFS.begin(true)) {
    Serial.println("ERROR: LittleFS mount failed!");
    return false;
  }

  // List files
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  Serial.println("LittleFS files:");
  while (file) {
    Serial.print("  ");
    Serial.print(file.name());
    Serial.print(" (");
    Serial.print(file.size());
    Serial.println(" bytes)");
    file = root.openNextFile();
  }

  return true;
}

// =============================================================================
// WebSocket Event Handler
// =============================================================================

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("[WS] Client #%u connected from %s\n", client->id(),
                     client->remoteIP().toString().c_str());
      ws_client_connected = true;
      // Send info message on connect
      ws_send_info(client);
      ws_send_status(client);
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
      ws_client_connected = (ws.count() > 0);
      // Safety: stop experiment if last client disconnects
      if (!ws_client_connected && experimentActive) {
        experimentActive = false;
        flag_pid = true;
        flag_step = true;
        flag_deadband = true;
        myPID.SetMode(MANUAL);
        u_distancia = 0;
        error_distancia = 0;
        MotorSpeed = 0;
        MotorDirection = 1;
        SetMotorControl();
        Serial.println("[SAFETY] Experiment stopped - all clients disconnected");
      }
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;  // Null-terminate
        ws_parse_command((char *)data, client);
      }
      break;
    }

    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// =============================================================================
// WebSocket Command Parser
// =============================================================================

void ws_parse_command(char *json_str, AsyncWebSocketClient *client) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json_str);

  if (error) {
    Serial.print("[WS] JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  const char *type = doc["t"];
  if (!type || strcmp(type, "cmd") != 0) return;

  const char *cmd = doc["c"];
  if (!cmd) return;

  Serial.printf("[WS] Command: %s", cmd);

  // PID parameter commands
  if (strcmp(cmd, "kp") == 0) {
    double val = doc["v"];
    Kp = constrain(val, 0.0, 100.0);
    pid_params_changed = true;
    Serial.printf(" = %.2f\n", Kp);
    ws_send_ack("kp", Kp);
  }
  else if (strcmp(cmd, "ki") == 0) {
    double val = doc["v"];
    Ki = constrain(val, 0.0, 50.0);
    pid_params_changed = true;
    Serial.printf(" = %.2f\n", Ki);
    ws_send_ack("ki", Ki);
  }
  else if (strcmp(cmd, "kd") == 0) {
    double val = doc["v"];
    Kd = constrain(val, 0.0, 50.0);
    pid_params_changed = true;
    Serial.printf(" = %.2f\n", Kd);
    ws_send_ack("kd", Kd);
  }
  else if (strcmp(cmd, "ref") == 0) {
    double val = doc["v"];
    x_ref = constrain(val, 1.0, 100.0);
    Serial.printf(" = %.1f\n", x_ref);
    ws_send_ack("ref", x_ref);
  }
  else if (strcmp(cmd, "deadband") == 0) {
    int val = doc["v"];
    deadband = constrain(val, 0, 500);
    Serial.printf(" = %d\n", deadband);
    ws_send_ack("deadband", (double)deadband);
  }
  else if (strcmp(cmd, "start") == 0) {
    const char *mode = doc["m"];
    if (mode && strcmp(mode, "step") == 0) {
      Serial.println(" -> START STEP");
      if (StepTimeDuration > 0 && StepAmplitude > 0) {
        currentExperimentMode = STEP_MODE;
        experimentActive = true;
        flag_step = false;
        tiempo_inicial_step = millis();
      } else {
        Serial.println("  ERROR: Set amplitude and duration first");
      }
    }
    else if (mode && strcmp(mode, "deadband") == 0) {
      Serial.println(" -> START DEADBAND");
      currentExperimentMode = DEADBAND_MODE;
      experimentActive = true;
      flag_deadband = false;
      tiempo_inicial_deadband = millis();
    }
    else {
      Serial.println(" -> START PID");
      currentExperimentMode = PID_MODE;
      experimentActive = true;
      flag_pid = false;
      tiempo_inicial_pid = millis();
      PIDMotorDirection = 1;
    }
    ws_send_status_all();
  }
  else if (strcmp(cmd, "stop") == 0) {
    Serial.println(" -> STOP");
    experimentActive = false;
    // Reset all mode flags
    flag_pid = true;
    flag_step = true;
    flag_deadband = true;
    myPID.SetMode(MANUAL);
    u_distancia = 0;
    error_distancia = 0;
    MotorSpeed = 0;
    MotorDirection = 1;
    SetMotorControl();
    ws_send_status_all();
  }
  // Step response parameter commands
  else if (strcmp(cmd, "step_amp") == 0) {
    double val = doc["v"];
    StepAmplitude = constrain(val, 0.0, v_batt);
    Serial.printf(" = %.1f V\n", StepAmplitude);
    ws_send_ack("step_amp", StepAmplitude);
  }
  else if (strcmp(cmd, "step_time") == 0) {
    double val = doc["v"];
    StepTimeDuration = constrain((uint32_t)(val * 1000), 0, 20000);
    Serial.printf(" = %.1f s (%d ms)\n", val, StepTimeDuration);
    ws_send_ack("step_time", StepTimeDuration / 1000.0);
  }
  else if (strcmp(cmd, "step_dir") == 0) {
    int val = doc["v"];
    StepMotorDirection = constrain(val, 0, 1);
    Serial.printf(" = %s\n", StepMotorDirection ? "Forward" : "Reverse");
    ws_send_ack("step_dir", (double)StepMotorDirection);
  }
  else if (strcmp(cmd, "vbatt") == 0) {
    double val = doc["v"];
    v_batt = constrain(val, 0.0, 12.0);
    Serial.printf(" = %.1f V\n", v_batt);
    ws_send_ack("vbatt", v_batt);
  }
  // Deadband calibration parameter commands
  else if (strcmp(cmd, "db_dir") == 0) {
    int val = doc["v"];
    DeadbandMotorDirection = constrain(val, 0, 1);
    Serial.printf(" = %s\n", DeadbandMotorDirection ? "Forward" : "Reverse");
    ws_send_ack("db_dir", (double)DeadbandMotorDirection);
  }
  else if (strcmp(cmd, "db_threshold") == 0) {
    double val = doc["v"];
    motion_threshold = constrain(val, 0.01, 1.0);
    Serial.printf(" = %.2f cm\n", motion_threshold);
    ws_send_ack("db_threshold", motion_threshold);
  }
  else if (strcmp(cmd, "db_apply") == 0) {
    if (calibrated_deadband > 0) {
      deadband = calibrated_deadband;
      Serial.printf(" -> Applied deadband = %d\n", deadband);
      ws_send_ack("deadband", (double)deadband);
    }
  }
  else if (strcmp(cmd, "params") == 0) {
    // Request current parameters
    Serial.println(" -> REQUEST PARAMS");
    ws_send_ack("kp", Kp);
    ws_send_ack("ki", Ki);
    ws_send_ack("kd", Kd);
    ws_send_ack("ref", x_ref);
    ws_send_ack("deadband", (double)deadband);
    ws_send_ack("step_amp", StepAmplitude);
    ws_send_ack("step_time", StepTimeDuration / 1000.0);
    ws_send_ack("step_dir", (double)StepMotorDirection);
    ws_send_ack("vbatt", v_batt);
    ws_send_ack("db_dir", (double)DeadbandMotorDirection);
    ws_send_ack("db_threshold", motion_threshold);
    ws_send_ack("db_result", (double)calibrated_deadband);
    ws_send_status(client);
  }
  else {
    Serial.println(" -> UNKNOWN");
  }
}

// =============================================================================
// WebSocket Broadcast Functions
// =============================================================================

void ws_send_pid_data() {
  if (ws.count() == 0) return;

  uint32_t ts = millis() - tiempo_inicial_pid;

  JsonDocument doc;
  doc["t"] = "d";
  doc["m"] = "pid";
  doc["ts"] = ts;
  doc["d"] = round(distancia * 100.0) / 100.0;
  doc["r"] = x_ref;
  doc["e"] = round(error_distancia * 100.0) / 100.0;
  doc["kp"] = Kp;
  doc["ki"] = Ki;
  doc["kd"] = Kd;
  doc["u"] = round(u_distancia * 10.0) / 10.0;
  doc["pwm"] = MotorSpeed;
  doc["dir"] = PIDMotorDirection;

  char buffer[256];
  size_t len = serializeJson(doc, buffer);
  ws.textAll(buffer, len);
  ws_packet_count++;
}

void ws_send_step_data() {
  if (ws.count() == 0) return;

  uint32_t ts = millis() - tiempo_inicial_step;

  JsonDocument doc;
  doc["t"] = "d";
  doc["m"] = "step";
  doc["ts"] = ts;
  doc["d"] = round(medi * 100.0) / 100.0;
  doc["dir"] = StepMotorDirection;
  doc["vb"] = v_batt;
  doc["amp"] = StepAmplitude;
  doc["pwm"] = MotorSpeed;
  doc["applied"] = appliedStepValue;

  char buffer[256];
  size_t len = serializeJson(doc, buffer);
  ws.textAll(buffer, len);
  ws_packet_count++;
}

void ws_send_deadband_data() {
  if (ws.count() == 0) return;

  uint32_t ts = millis() - tiempo_inicial_deadband;

  JsonDocument doc;
  doc["t"] = "d";
  doc["m"] = "deadband";
  doc["ts"] = ts;
  doc["pwm"] = MotorSpeed;
  doc["d"] = round(medi * 100.0) / 100.0;
  doc["d0"] = round(initial_distance * 100.0) / 100.0;
  doc["motion"] = motion_detected ? 1 : 0;

  char buffer[256];
  size_t len = serializeJson(doc, buffer);
  ws.textAll(buffer, len);
  ws_packet_count++;
}

void ws_send_ack(const char *cmd, double value) {
  JsonDocument doc;
  doc["t"] = "ack";
  doc["c"] = cmd;
  doc["v"] = round(value * 100.0) / 100.0;

  char buffer[128];
  size_t len = serializeJson(doc, buffer);
  ws.textAll(buffer, len);
}

void ws_send_status(AsyncWebSocketClient *client) {
  JsonDocument doc;
  doc["t"] = "status";
  const char *mode_names[] = {"pid", "step", "deadband"};
  doc["mode"] = mode_names[currentExperimentMode];
  doc["active"] = experimentActive;

  char buffer[128];
  size_t len = serializeJson(doc, buffer);

  if (client) {
    client->text(buffer, len);
  } else {
    ws.textAll(buffer, len);
  }
}

void ws_send_status_all() {
  ws_send_status(nullptr);
}

void ws_send_info(AsyncWebSocketClient *client) {
  JsonDocument doc;
  doc["t"] = "info";
  doc["id"] = train_id;
  doc["ip"] = "192.168.4.1";
  doc["ap"] = ap_ssid;
  doc["fw"] = FW_VERSION;

  char buffer[192];
  size_t len = serializeJson(doc, buffer);

  if (client) {
    client->text(buffer, len);
  } else {
    ws.textAll(buffer, len);
  }
}

// =============================================================================
// HTTP Server Setup
// =============================================================================

void setup_webserver() {
  // Serve static files from LittleFS
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // WebSocket handler
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Captive portal: any unmatched request (including OS connectivity-check URLs
  // such as /generate_204, /hotspot-detect.html, /ncsi.txt) redirects to the
  // dashboard, so joining the AP opens the app automatically.
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect("http://192.168.4.1/");
  });

  // Start server
  server.begin();
  Serial.println("HTTP server started on port 80");
  Serial.println("WebSocket endpoint: ws://192.168.4.1/ws");
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
  Serial.begin(115200);
  pinMode(STATUS_LED, OUTPUT);

  Serial.println("\n\n");
  Serial.println("==============================================");
  Serial.println("  MOBILE Train Control Firmware");
  Serial.println("  Version: " FW_VERSION);
  Serial.println("  Mode: WiFi AP + WebSocket Dashboard");
  Serial.println("  Modes: PID, Step Response, Deadband");
  Serial.println("==============================================");

  // Load configuration from EEPROM
  loadConfiguration();

  if (!is_configured) {
    Serial.println("\n*** TRAIN NOT CONFIGURED ***");
    enterConfigMode();
  }

  // Train is configured
  Serial.println("\n========================================");
  Serial.println("TRAIN CONFIGURATION LOADED");
  Serial.println("========================================");
  Serial.println("Train ID: " + train_id);
  Serial.println("========================================\n");

  // LittleFS Setup
  if (!setup_littlefs()) {
    Serial.println("FATAL: Cannot start without LittleFS!");
    Serial.println("Upload data/ folder using LittleFS upload tool.");
    while (true) {
      blinkLED(5, 100);
      delay(1000);
    }
  }

  // WiFi AP Setup
  setup_wifi_ap();

  // Web Server + WebSocket Setup
  setup_webserver();

  // Motor Setup
  setup_motor();

  // ToF Sensor Setup
  setup_ToF();

  // PID Configuration
  myPID.SetSampleTime(SampleTime);
  myPID.SetOutputLimits(umin, umax);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(MANUAL);

  Serial.println("\nPID Configuration:");
  Serial.print("  Sample Time: "); Serial.print(SampleTime); Serial.println("ms");
  Serial.print("  Output Limits: +/-"); Serial.println(umax);
  Serial.print("  Deadband: "); Serial.println(deadband);

  Serial.println("\n==============================================");
  Serial.println("Setup Complete! Ready for mobile control.");
  Serial.println("  1. Connect phone to WiFi: " + ap_ssid);
  Serial.println("  2. Password: " + String(ap_password));
  Serial.println("  3. Open browser: http://192.168.4.1");
  Serial.println("==============================================");

  // Solid LED on when ready
  digitalWrite(STATUS_LED, HIGH);
}

// =============================================================================
// Main Loop
// =============================================================================

void loop() {
  // Always check for serial config commands
  checkSerialConfig();

  // Captive portal DNS (redirects any lookup to the dashboard)
  dnsServer.processNextRequest();

  // Update status LED
  updateStatusLED();

  // Clean up WebSocket clients
  ws.cleanupClients();

  // Run experiment if active
  if (experimentActive) {
    switch (currentExperimentMode) {
      case PID_MODE:
        loop_pid_experiment();
        break;
      case STEP_MODE:
        loop_step_experiment();
        break;
      case DEADBAND_MODE:
        loop_deadband_experiment();
        break;
    }
  } else {
    // Idle mode - motor off
    MotorSpeed = 0;
    SetMotorControl();
    delay(100);
  }
}

// =============================================================================
// PID Experiment Loop
// =============================================================================

void loop_pid_experiment() {
  if (flag_pid == false) {
    flag_pid = true;
    tiempo_inicial_pid = millis();
    pidWarmupCounter = 0;
    ws_packet_count = 0;
    Serial.println("[PID] Experiment started!");
    Serial.print("  Sensor warm-up: "); Serial.print(PID_WARMUP_SAMPLES); Serial.println(" samples");
    myPID.SetMode(MANUAL);
    PIDMotorDirection = 1;
  }

  if (pid_params_changed) {
    myPID.SetTunings(Kp, Ki, Kd);
    pid_params_changed = false;
    Serial.printf("[PID] Params updated - Kp:%.2f Ki:%.2f Kd:%.2f\n", Kp, Ki, Kd);
  }

  read_ToF_sensor();

  // SENSOR WARM-UP: Discard first N readings
  if (pidWarmupCounter < PID_WARMUP_SAMPLES) {
    pidWarmupCounter++;
    MotorSpeed = 0;
    MotorDirection = PIDMotorDirection;
    SetMotorControl();
    if (pidWarmupCounter == PID_WARMUP_SAMPLES) {
      Serial.println("[PID] Warm-up complete, starting control...");
      myPID.SetMode(AUTOMATIC);
    }
    delay(SampleTime);
    return;
  }

  distancia = medi;
  error_distancia = x_ref - distancia;
  myPID.Compute();
  double u = u_distancia;

  if (distancia > 200) {
    if (MotorSpeed < 200) {
      MotorSpeed = MotorSpeed - 10;
    } else {
      MotorSpeed = 0;
    }
    SetMotorControl();
    ws_send_pid_data();
    delay(SampleTime);
    return;
  }

  if (abs(u) <= lim) {
    MotorSpeed = 0;
  }
  else if (u > lim) {
    PIDMotorDirection = 1;
    int deadband_comp = deadband;
    MotorSpeed = constrain(int(u + deadband_comp), 0, 1024);
  }
  else if (u < -lim) {
    PIDMotorDirection = 0;
    int deadband_comp = deadband;
    MotorSpeed = constrain(int(-u + deadband_comp), 0, 1024);
  }

  if ((abs(u) <= lim) && (abs(ponderado) <= 0.75)) {
    MotorSpeed = 0;
  }

  MotorDirection = PIDMotorDirection;
  SetMotorControl();
  ws_send_pid_data();
  delay(SampleTime);
}

// =============================================================================
// Step Response Experiment Loop
// =============================================================================

void loop_step_experiment() {
  if (flag_step == false) {
    flag_step = true;
    Serial.println("[STEP] Experiment started!");
    Serial.print("  Amplitude: "); Serial.print(StepAmplitude); Serial.println("V");
    Serial.print("  Duration: "); Serial.print(StepTimeDuration / 1000.0); Serial.println("s");
    Serial.print("  Direction: "); Serial.println(StepMotorDirection ? "Forward" : "Reverse");

    delta = millis() - tiempo_inicial_step;
    StepTime = tiempo_inicial_step + StepTimeDuration;

    stepWarmupCounter = 0;
    stepBaselineCounter = 0;
    appliedStepValue = 0.0;
    ws_packet_count = 0;
  }

  read_ToF_sensor();

  // Three-phase: WARMUP -> BASELINE -> STEP APPLIED
  if (stepWarmupCounter < STEP_WARMUP_SAMPLES) {
    stepWarmupCounter++;
    MotorSpeed = 0;
    appliedStepValue = 0.0;
    MotorDirection = StepMotorDirection;
    SetMotorControl();
    if (stepWarmupCounter == STEP_WARMUP_SAMPLES) {
      Serial.println("[STEP] Warm-up complete, collecting baseline...");
    }
    delay(21);
    return;
  }
  else if (stepBaselineCounter < STEP_BASELINE_SAMPLES) {
    stepBaselineCounter++;
    MotorSpeed = 0;
    appliedStepValue = 0.0;
    if (stepBaselineCounter == STEP_BASELINE_SAMPLES) {
      Serial.println("[STEP] Baseline collected, applying step!");
    }
  }
  else {
    appliedStepValue = StepAmplitude;
    u_step = StepAmplitude * 1024 / v_batt;
    MotorSpeed = constrain(u_step, 0, 1024);
  }

  MotorDirection = StepMotorDirection;
  SetMotorControl();
  ws_send_step_data();

  if (millis() >= StepTime) {
    Serial.println("[STEP] Experiment complete");
    flag_step = true;
    experimentActive = false;
    MotorSpeed = 0;
    SetMotorControl();
    ws_send_status_all();
  }

  delay(21);
}

// =============================================================================
// Deadband Calibration Experiment Loop
// =============================================================================

void loop_deadband_experiment() {
  if (flag_deadband == false) {
    flag_deadband = true;
    tiempo_inicial_deadband = millis();
    Serial.println("[DEADBAND] Calibration started!");
    Serial.print("  Direction: "); Serial.println(DeadbandMotorDirection ? "Forward" : "Reverse");
    Serial.print("  Motion threshold: "); Serial.print(motion_threshold); Serial.println(" cm");

    MotorSpeed = 0;
    calibrated_deadband = 0;
    motion_detected = false;
    consecutive_motion_count = 0;
    ws_packet_count = 0;

    // Get initial baseline (average of 10 readings)
    double sum = 0;
    for (int i = 0; i < 10; i++) {
      read_ToF_sensor();
      sum += medi;
      delay(20);
    }
    initial_distance = sum / 10.0;

    ledcAttach(Control_v, 5000, 10);
    SetMotorControl();

    Serial.print("  Initial distance: "); Serial.print(initial_distance, 2); Serial.println(" cm");
    Serial.println("  Ramping PWM until motion detected...");
  }

  // Read current distance (average of 3 readings)
  double sum = 0;
  for (int i = 0; i < 3; i++) {
    read_ToF_sensor();
    sum += medi;
    delay(5);
  }
  double current_distance = sum / 3.0;

  // Ramp PWM
  MotorSpeed += pwm_increment;
  MotorDirection = DeadbandMotorDirection;
  SetMotorControl();

  double deviation = abs(current_distance - initial_distance);

  ws_send_deadband_data();

  // Check for motion
  if (deviation >= motion_threshold && MotorSpeed > 30) {
    consecutive_motion_count++;

    if (consecutive_motion_count >= MOTION_CONFIRM_COUNT) {
      motion_detected = true;
      calibrated_deadband = MotorSpeed - MOTION_CONFIRM_COUNT;

      Serial.println("========================================");
      Serial.println("MOTION DETECTED!");
      Serial.print("  Deadband PWM: "); Serial.println(calibrated_deadband);
      Serial.print("  Initial: "); Serial.print(initial_distance, 2); Serial.println(" cm");
      Serial.print("  Current: "); Serial.print(current_distance, 2); Serial.println(" cm");
      Serial.print("  Deviation: "); Serial.print(deviation, 3); Serial.println(" cm");
      Serial.println("========================================");

      ws_send_deadband_data();
      delay(50);

      MotorSpeed = 0;
      SetMotorControl();

      // Send result via ack
      ws_send_ack("db_result", (double)calibrated_deadband);

      delay(500);
      experimentActive = false;
      flag_deadband = true;
      ws_send_status_all();
      return;
    }
  } else {
    if (deviation < motion_threshold * 0.5) {
      consecutive_motion_count = 0;
    }
  }

  // Progress report every 50 PWM
  if (MotorSpeed % 50 == 0) {
    Serial.print("  PWM: "); Serial.print(MotorSpeed);
    Serial.print(" - Distance: "); Serial.print(current_distance, 2);
    Serial.print(" cm (change: "); Serial.print(deviation, 3);
    Serial.println(" cm)");
  }

  delay(pwm_delay);

  // Safety timeout
  if (MotorSpeed >= max_pwm_test) {
    Serial.println("========================================");
    Serial.println("WARNING: Max PWM reached without motion!");
    Serial.println("========================================");

    calibrated_deadband = deadband;
    MotorSpeed = 0;
    SetMotorControl();

    ws_send_ack("db_result", (double)calibrated_deadband);

    experimentActive = false;
    flag_deadband = true;
    ws_send_status_all();
  }
}

// =============================================================================
// Sensor Reading
// =============================================================================

void read_ToF_sensor() {
  medi = 0;
  for (int i = 0; i < 2; i++) {
    uint16_t range = SensorToF.readReg16Bit(SensorToF.RESULT_RANGE_STATUS + 10);
    medi += range;
    delay(21);
  }
  medi = medi / 2;

  newSum = movingSum(arrNumbers, &sum, pos, len, medi);
  newAvg = newSum / (float)len;
  pos++;
  if (pos >= len) {
    pos = 0;
  }
  oldAvg = newAvg;
  medi = (0.8 * newAvg + 0.2 * oldAvg) / 10.0;
}

int movingSum(int *ptrArrNumbers, long *ptrSum, int pos, int len, int nextNum) {
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  ptrArrNumbers[pos] = nextNum;
  return *ptrSum;
}

// =============================================================================
// Motor Control Functions
// =============================================================================

void setup_motor() {
  pinMode(STBY, OUTPUT);
  pinMode(Control_fwd, OUTPUT);
  pinMode(Control_back, OUTPUT);
  pinMode(Control_v, OUTPUT);

  digitalWrite(STBY, HIGH);
  digitalWrite(Control_v, HIGH);

  ledcAttach(Control_v, 5000, 10);

  Serial.println("Motor initialized");
}

void SetMotorControl() {
  int effectiveDirection = motor_inverted ? !MotorDirection : MotorDirection;

  if (effectiveDirection == 1) {
    digitalWrite(Control_fwd, LOW);
    digitalWrite(Control_back, HIGH);
  } else {
    digitalWrite(Control_fwd, HIGH);
    digitalWrite(Control_back, LOW);
  }

  int pwm_value = constrain(MotorSpeed, 0, 1023);
  ledcWrite(Control_v, pwm_value);
}

// =============================================================================
// ToF Sensor Setup
// =============================================================================

void setup_ToF() {
  Serial.println("Initializing ToF sensor...");

  Wire.begin(10, 9);

  SensorToF.setTimeout(500);
  while (!SensorToF.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    delay(1000);
  }

  SensorToF.setSignalRateLimit(0.25);
  SensorToF.setMeasurementTimingBudget(22000);
  SensorToF.startContinuous();

  Serial.println("ToF sensor ready");
}
