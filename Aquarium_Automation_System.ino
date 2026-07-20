/*
  ESP32 Fish Tank Controller (WITH Telegram) — NTP (No RTC)
  ---------------------------------------------------------
  Sensors/Actuators:
    - Ultrasonic #1 (Water): TRIG=26, ECHO=25 (distance in cm)
    - Ultrasonic #2 (Food) : TRIG=5,  ECHO=18 (distance in cm)
    - Turbidity AO         : ADC34 (maps to % + label)
    - Relays               : OUTLET=22 (drain), INLET=23 (fill)
    - Servo                : GPIO14 (fish food dispenser)
    - Serial Monitor       : 115200
    - Time source          : Wi-Fi + NTP (Asia/Colombo, UTC+5:30)

  Behavior:
    Scenario 1 (Cloudiness): if turbidity% > THRESHOLD
      - OUTLET ON (drain) until TIMEOUT or SAFE_MIN_CM (protect fish),
      - then INLET ON (refill) until HIGH_LEVEL_CM is reached.
    Scenario 2 (Auto-refill): if water distance >= LOW_LEVEL_CM
      - INLET ON until distance <= HIGH_LEVEL_CM.

  Telegram Commands:
    /start
    /status
    /time
    /feednow
    /flushnow
    /get_turb
    /set_turb <0..100>
    /set_outlet_max <sec 1..600>
    /set_levels <low_cm> <high_cm>     (require low > high)
    /set_safe_min <cm>                 (must be > HIGH)
    /set_feed HH:MM HH:MM HH:MM
    /set_servo <open_deg> <rest_deg> <ms 200..3000>
    /logic low|high                    (relay polarity: ON=LOW or ON=HIGH)

  Libraries:
    - UniversalTelegramBot (needs ArduinoJson v6.x)
    - ESP32Servo
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <math.h>
#include <time.h>

// ---------------- Wi-Fi + Telegram credentials (yours) ----------------
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";
const char* BOT_TOKEN = "";
const String CHAT_ID  = "";  // String for UniversalTelegramBot
// ----------------------------------------------------------------------

// ---------------- Pins (as requested) ----------------
#define SERVO_PIN          14
#define TURBIDITY_PIN      34  // ADC1_CH6 (input-only)
#define TRIG_FOOD           5
#define ECHO_FOOD          18  // ⚠ level-shift 5V -> 3.3V
#define TRIG_WATER         26
#define ECHO_WATER         25  // ⚠ level-shift 5V -> 3.3V
#define RELAY_OUTLET       22  // pump-out (drain)
#define RELAY_INLET        23  // fill-in (refill)
#define LED_PIN             2
// -----------------------------------------------------

// --------------- Relay logic (low-profile default) ---------------
bool g_activeLow = true; // Active-LOW relays: ON=LOW, OFF=HIGH. Flip with /logic high if needed.
inline void relayWrite(uint8_t pin, bool on) {
  if (g_activeLow) digitalWrite(pin, on ? LOW : HIGH);
  else             digitalWrite(pin, on ? HIGH : LOW);
}
inline bool relayIsOn(uint8_t pin) {
  int lvl = digitalRead(pin);
  return g_activeLow ? (lvl == LOW) : (lvl == HIGH);
}
// -----------------------------------------------------------------

// ------------------- User Settings -------------------
int TANK_HEIGHT_CM = 15;   // info/display only

// Water level thresholds (cm; distance from sensor to water surface)
// Larger distance => lower water. Smaller distance => higher water.
int LOW_LEVEL_CM   = 12;   // start filling when distance >= 12 cm
int HIGH_LEVEL_CM  = 3;    // stop filling when distance <= 3 cm
int SAFE_MIN_CM    = 14;   // during drain: stop if distance >= 14 cm (protect fish)

// Turbidity calibration and threshold
int TURBID_CLEAN_MV  = 2600; // very clean -> ~0%
int TURBID_DIRTY_MV  = 1400; // very cloudy -> ~100%
int TURBID_THRESHOLD = 50;   // % to trigger Scenario 1

// Scenario 1: outlet safety cap
int OUTLET_MAX_SEC   = 45;   // max drain seconds

// Servo parameters (fish food)
int SERVO_FEED_ANGLE = 120;  // degrees
int SERVO_REST_ANGLE = 0;    // degrees
int SERVO_MOVE_MS    = 700;  // ms

// Feeding times (NTP local time, 24h)
struct FeedTime { uint8_t h; uint8_t m; };
FeedTime FEEDS[3] = { {6,40}, {12,30}, {18,30} };
// -----------------------------------------------------

// ----------------- Globals / Devices -----------------
WiFiClientSecure secureClient;
UniversalTelegramBot bot(BOT_TOKEN, secureClient);
Servo feeder;

const unsigned long ECHO_TIMEOUT_US = 30000UL; // ~5m for pulseIn
const unsigned long BOT_POLL_MS = 1200;
unsigned long lastPoll = 0;

// NTP: Asia/Colombo (UTC+5:30)
const long GMT_OFFSET = 19800; // seconds
const int  DST_OFFSET = 0;

bool timeIsValid() { return time(nullptr) > 1700000000; } // ~Nov 2023 epoch check

void waitForTime() {
  configTime(GMT_OFFSET, DST_OFFSET, "pool.ntp.org", "time.nist.gov");
  Serial.print("Syncing time via NTP");
  for (int i=0; i<60 && !timeIsValid(); i++) { Serial.print("."); delay(500); }
  Serial.println();
}
String weekdayName(int wday) {
  const char* names[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  if (wday<0 || wday>6) return "?"; return names[wday];
}
String formatNow() {
  time_t now = time(nullptr);
  struct tm lt; localtime_r(&now, &lt);
  char buf[40];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           lt.tm_year+1900, lt.tm_mon+1, lt.tm_mday,
           lt.tm_hour, lt.tm_min, lt.tm_sec);
  return String(buf) + " (" + weekdayName(lt.tm_wday) + ")";
}
String two(int v) { char b[8]; snprintf(b, sizeof(b), "%02d", v); return String(b); }
// -----------------------------------------------------

// ----------------- Helpers: sensors ------------------
float readUltrasonicCM(uint8_t trig, uint8_t echo) {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (dur == 0) return NAN;
  return dur / 58.0f; // convert echo time -> cm
}

int readTurbidityMV() {
  const int N = 20;
  long acc = 0;
  analogSetWidth(12);
  analogSetPinAttenuation(TURBIDITY_PIN, ADC_11db); // full-scale ~3.3V
  for (int i=0;i<N;i++) { acc += analogRead(TURBIDITY_PIN); delay(3); }
  int avg = acc / N;
  return (int)((avg * 3300L) / 4095L); // mV (assuming 3.3V ref)
}

int turbidityPercentFromMV(int mv) {
  if (mv >= TURBID_CLEAN_MV) return 0;
  if (mv <= TURBID_DIRTY_MV) return 100;
  float span = (float)(TURBID_CLEAN_MV - TURBID_DIRTY_MV);
  float pct  = 100.0f * (TURBID_CLEAN_MV - mv) / span;
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  return (int)(pct + 0.5f);
}

const char* turbidityLabel(int pct) {
  if (pct <= 10) return "Very clean";
  if (pct <= 30) return "Clean";
  if (pct <= 60) return "Cloudy";
  return "Very cloudy";
}
// -----------------------------------------------------

// ----------------- Feeding (NTP clock) ---------------
void feedOnce(const char* reason) {
  feeder.write(SERVO_FEED_ANGLE);
  delay(SERVO_MOVE_MS);
  feeder.write(SERVO_REST_ANGLE);
  delay(150);
  bot.sendMessage(CHAT_ID, String("🍽 Feeding: ") + reason, "");
}

// once-per-minute gate so a feed time only triggers once
bool feedGateOncePerMinute() {
  if (!timeIsValid()) return false;
  static uint32_t lastKey = 0;
  time_t now = time(nullptr);
  struct tm lt; localtime_r(&now, &lt);
  uint32_t key = ((lt.tm_year+1900)*10000UL + (lt.tm_mon+1)*100UL + lt.tm_mday)*100UL + lt.tm_hour*1UL + lt.tm_min;
  if (key == lastKey) return false;
  for (auto &ft : FEEDS) {
    if (lt.tm_hour==ft.h && lt.tm_min==ft.m) {
      lastKey = key;
      return true;
    }
  }
  return false;
}
// -----------------------------------------------------

// --------- Scenario 1 state machine (cloudiness) -----
enum FlushState { FLUSH_IDLE, FLUSH_DRAIN, FLUSH_FILL };
FlushState flushState = FLUSH_IDLE;
unsigned long outletStartMs = 0;

void startDrain() {
  flushState = FLUSH_DRAIN;
  outletStartMs = millis();
  relayWrite(RELAY_INLET, false);
  relayWrite(RELAY_OUTLET, true);
  bot.sendMessage(CHAT_ID, "🌊 Cloudy → OUTLET ON (draining)...", "");
}

void stopDrainStartFill(const char* why) {
  relayWrite(RELAY_OUTLET, false);
  relayWrite(RELAY_INLET, true);
  flushState = FLUSH_FILL;
  bot.sendMessage(CHAT_ID, String("↗ Drain stop (") + why + "). INLET ON (refill)...", "");
}

void finishFlush() {
  relayWrite(RELAY_INLET, false);
  flushState = FLUSH_IDLE;
  bot.sendMessage(CHAT_ID, "✅ Cloudiness cycle complete (refilled to HIGH).", "");
}
// -----------------------------------------------------

// ---------------- Telegram utilities -----------------
bool parseHHMM(const String& in, uint8_t &hh, uint8_t &mm) {
  int H=-1, M=-1;
  if (sscanf(in.c_str(), "%d:%d", &H, &M) == 2 && H>=0 && H<=23 && M>=0 && M<=59) {
    hh = (uint8_t)H; mm = (uint8_t)M; return true;
  }
  return false;
}

void sendStatus() {
  float w = readUltrasonicCM(TRIG_WATER, ECHO_WATER);
  float f = readUltrasonicCM(TRIG_FOOD,  ECHO_FOOD);
  int   mv = readTurbidityMV();
  int   p  = turbidityPercentFromMV(mv);

  String s;
  s += "⏱ Time: ";
  s += timeIsValid() ? formatNow() : String("(not synced yet)");
  s += "\n🏷 Tank: " + String(TANK_HEIGHT_CM) + " cm";
  s += "\n💧 Water: " + (isnan(w)? String("NaN") : String(w,1) + " cm");
  s += " | LOW=" + String(LOW_LEVEL_CM) + " HIGH=" + String(HIGH_LEVEL_CM) + " SAFE_MIN=" + String(SAFE_MIN_CM) + " cm";
  s += "\n🍘 Food : " + (isnan(f)? String("NaN") : String(f,1) + " cm");
  s += "\n🧪 Turbidity: " + String(p) + "% (" + turbidityLabel(p) + ") [" + String(mv) + " mV], THRESH=" + String(TURBID_THRESHOLD) + "%";
  s += "\n🔌 Relays: OUT=" + String(relayIsOn(RELAY_OUTLET) ? "ON" : "OFF");
  s += " IN=" + String(relayIsOn(RELAY_INLET) ? "ON" : "OFF");
  s += " | Logic=" + String(g_activeLow ? "active-LOW" : "active-HIGH");
  s += "\n🌊 Cycle: " + String(flushState==FLUSH_IDLE?"idle":(flushState==FLUSH_DRAIN?"drain":"fill"));
  bot.sendMessage(CHAT_ID, s, "");
}

void handleMessage(int i) {
  String chat_id = bot.messages[i].chat_id;
  String text    = bot.messages[i].text;
  if (chat_id != CHAT_ID) { bot.sendMessage(chat_id, "Private bot. ✋", ""); return; }

  String t = text; t.trim(); t.toLowerCase();

  if (t == "/start") {
    bot.sendMessage(chat_id,
      "🐟 ESP32 Tank Controller (NTP)\n"
      "/status\n"
      "/time\n"
      "/feednow\n"
      "/flushnow\n"
      "/get_turb | /set_turb <0..100>\n"
      "/set_outlet_max <sec>\n"
      "/set_levels <low_cm> <high_cm>  (low>high)\n"
      "/set_safe_min <cm>\n"
      "/set_feed HH:MM HH:MM HH:MM\n"
      "/set_servo <open_deg> <rest_deg> <ms>\n"
      "/logic low|high\n", "");
  }
  else if (t == "/status") { sendStatus(); }
  else if (t == "/time")   { bot.sendMessage(CHAT_ID, timeIsValid()? ("⏱ "+formatNow()) : "Time not synced yet.", ""); }
  else if (t == "/feednow") { feedOnce("manual"); }
  else if (t == "/flushnow") {
    float w = readUltrasonicCM(TRIG_WATER, ECHO_WATER);
    if (!isnan(w) && flushState==FLUSH_IDLE) startDrain();
  }
  else if (t == "/get_turb") {
    bot.sendMessage(chat_id, "Turbidity threshold: " + String(TURBID_THRESHOLD) + "%", "");
  }
  else if (t.startsWith("/set_turb ")) {
    int val=-1; if (sscanf(text.c_str(), "/set_turb %d", &val)==1 && val>=0 && val<=100) {
      TURBID_THRESHOLD = val; bot.sendMessage(chat_id, "✅ turbidity threshold set.", "");
    } else bot.sendMessage(chat_id, "Usage: /set_turb 0..100", "");
  }
  else if (t.startsWith("/set_outlet_max ")) {
    int sec=-1; if (sscanf(text.c_str(), "/set_outlet_max %d", &sec)==1 && sec>0 && sec<=600) {
      OUTLET_MAX_SEC = sec; bot.sendMessage(chat_id, "✅ outlet max seconds set.", "");
    } else bot.sendMessage(chat_id, "Usage: /set_outlet_max <1..600>", "");
  }
  else if (t.startsWith("/set_levels ")) {
    int low=-1, high=-1;
    if (sscanf(text.c_str(), "/set_levels %d %d", &low, &high)==2 && low>high && low<200 && high>0) {
      LOW_LEVEL_CM = low; HIGH_LEVEL_CM = high;
      bot.sendMessage(chat_id, "✅ levels set.", "");
    } else bot.sendMessage(chat_id, "Usage: /set_levels <low_cm> <high_cm>  (low>high)", "");
  }
  else if (t.startsWith("/set_safe_min ")) {
    int cm=-1; if (sscanf(text.c_str(), "/set_safe_min %d", &cm)==1 && cm>HIGH_LEVEL_CM) {
      SAFE_MIN_CM = cm; bot.sendMessage(chat_id, "✅ SAFE_MIN set.", "");
    } else bot.sendMessage(chat_id, "Usage: /set_safe_min <cm> (must be > HIGH level)", "");
  }
  else if (t.startsWith("/set_feed ")) {
    // /set_feed HH:MM HH:MM HH:MM
    String a,b,c; int p1=t.indexOf(' ');
    if (p1>0) {
      String rest = text.substring(p1+1);
      int p2 = rest.indexOf(' ');
      int p3 = rest.lastIndexOf(' ');
      if (p2>0 && p3>p2) { a=rest.substring(0,p2); b=rest.substring(p2+1,p3); c=rest.substring(p3+1); }
      uint8_t h1,m1,h2,m2,h3,m3;
      if (parseHHMM(a,h1,m1) && parseHHMM(b,h2,m2) && parseHHMM(c,h3,m3)) {
        FEEDS[0]={h1,m1}; FEEDS[1]={h2,m2}; FEEDS[2]={h3,m3};
        bot.sendMessage(chat_id, "✅ feed times updated.", "");
      } else bot.sendMessage(chat_id, "Usage: /set_feed HH:MM HH:MM HH:MM", "");
    } else bot.sendMessage(chat_id, "Usage: /set_feed HH:MM HH:MM HH:MM", "");
  }
  else if (t.startsWith("/set_servo ")) {
    int open=-1, rest=-1, ms=-1;
    if (sscanf(text.c_str(), "/set_servo %d %d %d", &open, &rest, &ms)==3 &&
        open>=0 && open<=180 && rest>=0 && rest<=180 && ms>=200 && ms<=3000) {
      SERVO_FEED_ANGLE=open; SERVO_REST_ANGLE=rest; SERVO_MOVE_MS=ms;
      feeder.write(SERVO_REST_ANGLE);
      bot.sendMessage(chat_id, "✅ servo params updated.", "");
    } else bot.sendMessage(chat_id, "Usage: /set_servo <open_deg 0..180> <rest_deg 0..180> <ms 200..3000>", "");
  }
  else if (t.startsWith("/logic ")) {
    if (t.endsWith("low"))  { g_activeLow=true;  relayWrite(RELAY_INLET,false); relayWrite(RELAY_OUTLET,false); bot.sendMessage(chat_id,"Logic set LOW (ON=LOW). Both OFF.",""); }
    else if (t.endsWith("high")) { g_activeLow=false; relayWrite(RELAY_INLET,false); relayWrite(RELAY_OUTLET,false); bot.sendMessage(chat_id,"Logic set HIGH (ON=HIGH). Both OFF.",""); }
    else bot.sendMessage(chat_id,"Usage: /logic low|high","");
  }
  else {
    bot.sendMessage(chat_id, "Unknown. Try /start", "");
  }
}
// ----------------------------------------------------

// ----------------------- Setup -----------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Relays default OFF
  pinMode(RELAY_OUTLET, OUTPUT);
  pinMode(RELAY_INLET,  OUTPUT);
  relayWrite(RELAY_OUTLET, false);
  relayWrite(RELAY_INLET,  false);

  // Ultrasonic pins
  pinMode(TRIG_WATER, OUTPUT); digitalWrite(TRIG_WATER, LOW);
  pinMode(ECHO_WATER, INPUT);
  pinMode(TRIG_FOOD,  OUTPUT); digitalWrite(TRIG_FOOD, LOW);
  pinMode(ECHO_FOOD,  INPUT);

  // Servo
  feeder.setPeriodHertz(50);
  feeder.attach(SERVO_PIN, 500, 2400);
  feeder.write(SERVO_REST_ANGLE);

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Wi-Fi");
  int tries=0; while (WiFi.status()!=WL_CONNECTED && tries<60){ delay(250); Serial.print("."); tries++; }
  Serial.println(WiFi.status()==WL_CONNECTED? "\n✅ Wi-Fi connected" : "\n❌ Wi-Fi failed");
  if (WiFi.status()==WL_CONNECTED) { Serial.print("IP: "); Serial.println(WiFi.localIP()); }

  // NTP
  waitForTime();
  if (timeIsValid()) Serial.println(String("✅ NTP synced: ")+formatNow());
  else               Serial.println("⚠ NTP not ready (will keep trying).");

  // Telegram TLS: keep simple → skip cert validation
  secureClient.setInsecure();

  bot.sendMessage(CHAT_ID, "🐟 Tank controller (NTP) booted. /start", "");
}
// ----------------------------------------------------

// ------------------------ Loop -----------------------
void loop() {
  // --- Telegram polling ---
  if (millis() - lastPoll > BOT_POLL_MS) {
    int n = bot.getUpdates(bot.last_message_received + 1);
    while (n) { for (int i=0;i<n;i++) handleMessage(i); n = bot.getUpdates(bot.last_message_received + 1); }
    lastPoll = millis();
  }

  // --- Scheduled feeding (NTP local time) ---
  if (feedGateOncePerMinute()) {
    feedOnce("scheduled");
  }

  // --- Sense + control each ~1s ---
  static unsigned long lastSense=0;
  if (millis() - lastSense >= 1000) {
    lastSense = millis();

    float waterCM = readUltrasonicCM(TRIG_WATER, ECHO_WATER);
    int   mv      = readTurbidityMV();
    int   cloud   = turbidityPercentFromMV(mv);

    // Scenario 1: turbidity high → start drain if idle
    if (flushState == FLUSH_IDLE && !isnan(waterCM) && cloud > TURBID_THRESHOLD) {
      startDrain();
    }

    // Drain phase (timeout or safe level)
    if (flushState == FLUSH_DRAIN) {
      bool timeout = (millis() - outletStartMs) >= (unsigned long)OUTLET_MAX_SEC*1000UL;
      bool safeReached = (!isnan(waterCM) && waterCM >= SAFE_MIN_CM);
      if (timeout || safeReached) {
        stopDrainStartFill(timeout ? "timeout" : "safe level");
      }
    }

    // Fill phase (after drain) → until HIGH level
    if (flushState == FLUSH_FILL) {
      if (!isnan(waterCM) && waterCM <= HIGH_LEVEL_CM) {
        finishFlush();
      }
    }

    // Scenario 2: normal auto top-up (only when not flushing)
    if (flushState == FLUSH_IDLE && !isnan(waterCM)) {
      if (waterCM >= LOW_LEVEL_CM && !relayIsOn(RELAY_INLET)) {
        relayWrite(RELAY_OUTLET, false);
        relayWrite(RELAY_INLET,  true);
      } else if (waterCM <= HIGH_LEVEL_CM && relayIsOn(RELAY_INLET)) {
        relayWrite(RELAY_INLET, false);
      }
    }

    // Periodic debug ( ~3s )
    static unsigned long lastPrint=0;
    if (millis() - lastPrint >= 3000) {
      float foodCM = readUltrasonicCM(TRIG_FOOD, ECHO_FOOD);
      Serial.printf("[SENSE] Tank=%d cm | Water=%s cm (LOW=%d HIGH=%d SAFE=%d) | Food=%s cm | Turb=%d mV (%d%% %s) | OUT=%d IN=%d | state=%d | time=%s\n",
        TANK_HEIGHT_CM,
        isnan(waterCM) ? "NaN" : String(waterCM,1).c_str(),
        LOW_LEVEL_CM, HIGH_LEVEL_CM, SAFE_MIN_CM,
        isnan(foodCM) ? "NaN" : String(foodCM,1).c_str(),
        mv, cloud, turbidityLabel(cloud),
        relayIsOn(RELAY_OUTLET), relayIsOn(RELAY_INLET), (int)flushState,
        timeIsValid() ? "ok" : "ns"); // ns = not synced
      lastPrint = millis();
    }
  }

  // Optional: background re-try NTP until valid
  static unsigned long lastTimeCheck = 0;
  if (!timeIsValid() && millis() - lastTimeCheck > 5000) {
    waitForTime();
    lastTimeCheck = millis();
  }
}
// ----------------------------------------------------
