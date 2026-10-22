// ota_github.ino - OTA desde GitHub Releases (ESP32-S3, 16 MB, PSRAM)
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <mbedtls/sha256.h>
#include <esp_ota_ops.h>

static const char* OTA_MANIFEST_URL =
  "https://github.com/MarceloLagosS/pl-toon-codes/releases/latest/download/version.json";

// Pega aquí las raíces obtenidas con openssl (github.com y objects.githubusercontent.com)
static const char* GITHUB_ROOT_CA = R"CERT(
-----BEGIN CERTIFICATE-----
MIIDRjCCAsugAwIBAgIQGp6v7G3o4ZtcGTFBto2Q3TAKBggqhkjOPQQDAzCBiDEL
MAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNl
eSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMT
JVVTRVJUcnVzdCBFQ0MgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMjEwMzIy
MDAwMDAwWhcNMzgwMTE4MjM1OTU5WjBfMQswCQYDVQQGEwJHQjEYMBYGA1UEChMP
U2VjdGlnbyBMaW1pdGVkMTYwNAYDVQQDEy1TZWN0aWdvIFB1YmxpYyBTZXJ2ZXIg
QXV0aGVudGljYXRpb24gUm9vdCBFNDYwdjAQBgcqhkjOPQIBBgUrgQQAIgNiAAR2
+pmpbiDt+dd34wc7qNs9Xzjoq1WmVk/WSOrsfy2qw7LFeeyZYX8QeccCWvkEN/U0
NSt3zn8gj1KjAIns1aeibVvjS5KToID1AZTc8GgHHs3u/iVStSBDHBv+6xnOQ6Oj
ggEgMIIBHDAfBgNVHSMEGDAWgBQ64QmG1M8ZwpZ2dEl23OA1xmNjmjAdBgNVHQ4E
FgQU0SLaTFnxS18mOKqd1u7rDcP7qWEwDgYDVR0PAQH/BAQDAgGGMA8GA1UdEwEB
/wQFMAMBAf8wHQYDVR0lBBYwFAYIKwYBBQUHAwEGCCsGAQUFBwMCMBEGA1UdIAQK
MAgwBgYEVR0gADBQBgNVHR8ESTBHMEWgQ6BBhj9odHRwOi8vY3JsLnVzZXJ0cnVz
dC5jb20vVVNFUlRydXN0RUNDQ2VydGlmaWNhdGlvbkF1dGhvcml0eS5jcmwwNQYI
KwYBBQUHAQEEKTAnMCUGCCsGAQUFBzABhhlodHRwOi8vb2NzcC51c2VydHJ1c3Qu
Y29tMAoGCCqGSM49BAMDA2kAMGYCMQCMCyBit99vX2ba6xEkDe+YO7vC0twjbkv9
PKpqGGuZ61JZryjFsp+DFpEclCVy4noCMQCwvZDXD/m2Ko1HA5Bkmz7YQOFAiNDD
49IWa2wdT7R3DtODaSXH/BiXv8fwB9su4tU=
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
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
-----END CERTIFICATE-----
)CERT";

// ---------- Credenciales de la red STA (NVS, namespace "sta") ----------
Preferences staPrefs;
String sta_ssid = "";
String sta_pass = "";

void sta_load() {
  staPrefs.begin("sta", true);
  sta_ssid = staPrefs.getString("ssid", "");
  sta_pass = staPrefs.getString("pass", "");
  staPrefs.end();
}

void sta_save(const String& ssid, const String& pass) {
  staPrefs.begin("sta", false);
  staPrefs.putString("ssid", ssid);
  staPrefs.putString("pass", pass);
  staPrefs.end();
}

// Conecta la STA manteniendo el AP activo. El canal del AP pasará a ser el del router.
bool sta_connect(uint32_t timeout_ms) {
  if (sta_ssid.isEmpty()) return false;
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(sta_ssid.c_str(), sta_pass.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) {
    delay(200);
  }
  return WiFi.status() == WL_CONNECTED;
}

// Comandos serie: SET_WIFI:ssid:password (la contraseña puede contener ':')
// Llama a esta función desde el parser serie existente. Devuelve true si el comando era suyo.
bool handle_sta_command(const String& cmd) {
  if (cmd.startsWith("SET_WIFI:")) {
    String rest = cmd.substring(9);
    int sep = rest.indexOf(':');
    if (sep <= 0) { Serial.println("[STA] Format: SET_WIFI:ssid:password"); return true; }
    sta_save(rest.substring(0, sep), rest.substring(sep + 1));
    sta_load();
    Serial.println("[STA] Credentials saved. Reboot to conect.");
    return true;
  }
  if (cmd == "OTA_CHECK") {
    if (WiFi.status() != WL_CONNECTED) { Serial.println("[OTA] No STA connection"); return true; }
    ota_check_and_update();
    return true;
  }
  return false;
}

bool ota_fetch_manifest(OtaManifest& m) {
  WiFiClientSecure client;
  client.setCACert(GITHUB_ROOT_CA);
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(10000);
  if (!http.begin(client, OTA_MANIFEST_URL)) return false;
  Serial.printf("[TIME] epoch=%ld\n", (long)time(nullptr));
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    char tls_err[120] = {0};
    client.lastError(tls_err, sizeof(tls_err));   // motivo real del fallo TLS
    Serial.printf("[OTA] GET fallo: %d (%s) TLS: %s\n",
                  code, http.errorToString(code).c_str(), tls_err);
    http.end();
    return false;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, http.getStream());
  http.end();
  if (err) return false;

  m.version = doc["version"] | "";
  m.app_url = doc["app"]["url"] | "";
  m.app_sha = doc["app"]["sha256"] | "";
  m.fs_url  = doc["fs"]["url"]  | "";
  m.fs_sha  = doc["fs"]["sha256"] | "";
  return m.version.length() > 0 && m.app_url.length() > 0 && m.app_sha.length() == 64;
}

void ota_free(OtaImage& img) {
  if (img.data) free(img.data);
  img.data = nullptr;
  img.size = 0;
}

bool ota_stage(const String& url, const String& sha_hex, OtaImage& img) {
  WiFiClientSecure client;
  client.setCACert(GITHUB_ROOT_CA);
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(15000);
  if (!http.begin(client, url)) return false;
  if (http.GET() != HTTP_CODE_OK) { http.end(); return false; }

  int total = http.getSize();
  if (total <= 0) { http.end(); return false; }

  img.data = (uint8_t*)ps_malloc(total);
  if (!img.data) { http.end(); Serial.println("[OTA] Without PSRAM for the image"); return false; }

  WiFiClient* s = http.getStreamPtr();
  size_t got = 0;
  uint32_t last_rx = millis();
  bool ok = true;
  while (got < (size_t)total) {
    size_t avail = s->available();
    if (avail) {
      size_t n = s->readBytes(img.data + got, min(avail, (size_t)(total - got)));
      got += n;
      last_rx = millis();
    } else if (!s->connected() || millis() - last_rx > 15000) {
      ok = false;
      break;
    } else {
      delay(1);
    }
  }
  http.end();
  if (!ok || got != (size_t)total) { ota_free(img); return false; }
  img.size = got;

  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);            // 0 = SHA-256 (no SHA-224)
  mbedtls_sha256_update(&ctx, img.data, img.size);
  uint8_t digest[32];
  mbedtls_sha256_finish(&ctx, digest);
  mbedtls_sha256_free(&ctx);

  char hex[65];
  for (int i = 0; i < 32; i++) sprintf(hex + 2 * i, "%02x", digest[i]);
  if (!sha_hex.equalsIgnoreCase(hex)) {
    Serial.printf("[OTA] SHA-256 no coincide: %s\n", hex);
    ota_free(img);
    return false;
  }
  return true;
}

// ---------- Escritura en la partición (commit al terminar) ----------
bool ota_flash(OtaImage& img, int cmd) {
  if (cmd == U_SPIFFS) LittleFS.end();     // desmontar antes de sobrescribir
  if (!Update.begin(img.size, cmd)) {
    Serial.println(Update.errorString());
    return false;
  }
  if (Update.write(img.data, img.size) != img.size) {
    Serial.println(Update.errorString());
    Update.abort();
    return false;
  }
  return Update.end(true);
}

// ---------- Flujo principal ----------
void ota_check_and_update() {
  if (experimentActive || MotorSpeed != 0) {
    Serial.println("[OTA] Experiment in progress or active engine: omitted");
    return;
  }

  OtaManifest m;
  if (!ota_fetch_manifest(m)) { Serial.println("[OTA] Couldn't read the manifest"); return; }
  if (m.version == FW_VERSION) { Serial.println("[OTA] Firmware is already at version " FW_VERSION); return; }

  Serial.printf("[OTA] Available version %s (actual %s)\n", m.version.c_str(), FW_VERSION);

  // Parar el motor antes de cualquier escritura
  MotorSpeed = 0;
  SetMotorControl();

  OtaImage app, fs;
  bool staged = ota_stage(m.app_url, m.app_sha, app);
  if (staged && m.fs_url.length() > 0) {
    staged = ota_stage(m.fs_url, m.fs_sha, fs);
  }
  if (!staged) {
    Serial.println("[OTA] Failed download or verification: nothing has been written");
    ota_free(app);
    ota_free(fs);
    return;
  }

  // Punto de no retorno: la app queda marcada como arranque. Si falla antes, la app
  // activa no cambia.
  if (!ota_flash(app, U_FLASH)) {
    Serial.println("[OTA] Failure while writing the app: no change has been made");
    ota_free(app);
    ota_free(fs);
    return;
  }
  ota_free(app);

  if (fs.size > 0) {
    if (!ota_flash(fs, U_SPIFFS)) {
      // Caso raro: la app nueva ya está activa con la UI antigua. Ver notas.
      Serial.println("[OTA] ERROR while writing LittleFS after the app");
    }
    ota_free(fs);
  }

  Serial.println("[OTA] Update completed. Rebooting...");
  delay(1000);
  ESP.restart();
}