// ==========================================================
// GESTURAL MIDI GLOVE — RECEIVER (v7)
//
// To the Bela (USB serial, 115200):
//   LEFT,<fingers>,<roll>,<motion>      every 20 ms (zeros until that glove is READY)
//   RIGHT,<fingers>,<roll>,<motion>
//   STATUS,<LEFT|RIGHT>,<OFFLINE|WAITING|PROMPT|MEASURING|READY>,<step>
//   EVENT,<LEFT|RIGHT>,<name>
//   INFO,<text>                          messages (startup reason, command replies)
//
// From the Bela:
//   CMD,<SETUP|FULL>,<LEFT|RIGHT|BOTH>   start re-zero or full calibration
//
// To the gloves (ESP-NOW broadcast): whose turn it is, and setup requests.
// ==========================================================

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>

#define WIFI_CHANNEL         1
#define OUTPUT_INTERVAL_MS   20
#define CONTROL_INTERVAL_MS  100
#define STATUS_INTERVAL_MS   1000
#define TIMEOUT_MS           500
#define RX_DEBUG             0     // 1 = once-per-second '#' status line

// Lower radio power = smaller current spikes on the Bela's USB supply.
// Plenty for gloves within a few metres. If gloves drop out at your
// stage distance, raise to WIFI_POWER_15dBm or WIFI_POWER_19_5dBm.
#define RADIO_TX_POWER       WIFI_POWER_11dBm

// ---- glove packet (must match the gloves) ----
#define PACKET_MAGIC     0xA7
#define PACKET_VERSION   4

struct __attribute__((packed)) GloveData {
  uint8_t  magic;
  uint8_t  version;
  uint8_t  handID;
  uint8_t  fingers;
  uint8_t  flags;
  int16_t  roll;
  uint8_t  motion;
  uint8_t  setupState;
  uint8_t  setupStep;
  uint8_t  eventCode;
  uint8_t  eventSeq;
  uint32_t sequence;
};

// ---- control packet to the gloves ----
#define CTRL_MAGIC    0xA8
#define CTRL_VERSION  2
#define TURN_NONE     255

struct __attribute__((packed)) ControlData {
  uint8_t  magic;
  uint8_t  version;
  uint8_t  turn;          // 0 = left, 1 = right, 255 = none
  uint8_t  handState[2];  // setup state of each glove
  uint16_t session;       // changes every time the receiver restarts
  uint8_t  cmdSeq[2];     // increments when setup is requested for that glove
  uint8_t  cmdType[2];    // 1 = re-zero setup, 2 = full calibration
};

enum { ST_OFFLINE = 0, ST_WAITING, ST_PROMPT, ST_MEASURING, ST_READY };

const char *stateName(uint8_t s) {
  switch (s) {
    case ST_WAITING:   return "WAITING";
    case ST_PROMPT:    return "PROMPT";
    case ST_MEASURING: return "MEASURING";
    case ST_READY:     return "READY";
    default:           return "OFFLINE";
  }
}

const char *eventName(uint8_t e) {
  switch (e) {
    case 1: return "FULL_MODE";
    case 2: return "CAPTURE";
    case 3: return "STEP_DONE";
    case 4: return "REDO";
    case 5: return "RESTART";
    case 6: return "HAND_READY";
    case 7: return "RESUMED";
    default: return nullptr;
  }
}

const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWER-ON";
    case ESP_RST_EXT:       return "EXTERNAL PIN";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "CRASH (PANIC)";
    case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG";
    case ESP_RST_TASK_WDT:  return "TASK WATCHDOG";
    case ESP_RST_WDT:       return "WATCHDOG";
    case ESP_RST_DEEPSLEEP: return "DEEP SLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (power dip)";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

const char *handName[2] = {"LEFT", "RIGHT"};
const uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---- shared with the receive callback ----
struct HandState {
  GloveData data;
  unsigned long lastSeen;
  uint32_t packets;
  bool everSeen;
  bool rebooted;
};

HandState hands[2] = {};
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t rxRejected = 0;

// ---- main loop only ----
bool seenInLoop[2] = {false, false};
uint8_t lastEventSeq[2] = {0, 0};
uint8_t lastReportedState[2] = {255, 255};
uint8_t lastReportedStep[2] = {255, 255};
unsigned long lastStatusMs[2] = {0, 0};

uint16_t session = 0;
uint8_t cmdSeq[2] = {0, 0};
uint8_t cmdType[2] = {0, 0};

esp_reset_reason_t bootReason;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incoming, int len) {
  if (len != sizeof(GloveData)) return;

  GloveData p;
  memcpy(&p, incoming, sizeof(p));
  if (p.magic != PACKET_MAGIC || p.version != PACKET_VERSION || p.handID > 1) {
    rxRejected++;
    return;
  }

  unsigned long now = millis();
  portENTER_CRITICAL(&dataMux);
  HandState &h = hands[p.handID];
  if (h.everSeen && p.sequence < h.data.sequence) h.rebooted = true;
  h.data = p;
  h.lastSeen = now;
  h.packets++;
  h.everSeen = true;
  portEXIT_CRITICAL(&dataMux);
}

// ---- commands from the Bela ----

void handleCommand(char *line) {
  char *a = strtok(line, ",");
  char *b = strtok(nullptr, ",");
  char *c = strtok(nullptr, ",");
  if (!a || !b || !c || strcmp(a, "CMD") != 0) return;

  uint8_t type = 0;
  if (strcmp(b, "SETUP") == 0) type = 1;
  else if (strcmp(b, "FULL") == 0) type = 2;
  if (type == 0) return;

  bool left  = strcmp(c, "LEFT") == 0  || strcmp(c, "BOTH") == 0;
  bool right = strcmp(c, "RIGHT") == 0 || strcmp(c, "BOTH") == 0;
  if (!left && !right) return;

  if (left)  { cmdType[0] = type; cmdSeq[0]++; }
  if (right) { cmdType[1] = type; cmdSeq[1]++; }

  Serial.printf("INFO,%s requested for %s\n",
                type == 2 ? "Full calibration" : "Re-zero setup", c);
}

void readCommands() {
  static char buf[64];
  static int len = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handleCommand(buf);
        len = 0;
      }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = ch;
    } else {
      len = 0;   // too long: discard
    }
  }
}

void setup() {
  bootReason = esp_reset_reason();

  Serial.setTxBufferSize(1024);   // room for bursts, so printing never stalls
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setTxPower(RADIO_TX_POWER);
  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("INFO,ERROR: ESP-NOW init failed - restarting");
    delay(2000);
    ESP.restart();
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddress, 6);
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  esp_now_register_recv_cb(onDataRecv);

  session = (uint16_t)esp_random();

  Serial.printf("INFO,Receiver started (reset reason: %s)\n", resetReasonName(bootReason));

#if RX_DEBUG
  Serial.printf("# Receiver ready | MAC %s | channel %d\n",
                WiFi.macAddress().c_str(), WiFi.channel());
#endif
}

// Left goes first; a glove that is mid-setup keeps its turn
uint8_t computeTurn(const uint8_t st[2]) {
  for (int i = 0; i < 2; i++)
    if (st[i] == ST_PROMPT || st[i] == ST_MEASURING) return i;
  for (int i = 0; i < 2; i++)
    if (st[i] == ST_WAITING) return i;
  return TURN_NONE;
}

void printHand(int i, const GloveData &d, bool ready) {
  if (!ready) {
    Serial.printf("%s,0000,0,0\n", handName[i]);
    return;
  }
  Serial.printf("%s,%d%d%d%d,%d,%d\n", handName[i],
                (d.fingers >> 3) & 1, (d.fingers >> 2) & 1,
                (d.fingers >> 1) & 1, d.fingers & 1,
                (int)d.roll, (int)d.motion);
}

void loop() {
  static unsigned long lastOutput = 0, lastControl = 0;
  static bool repeatedBootInfo = false;

  readCommands();

  HandState copy[2];
  portENTER_CRITICAL(&dataMux);
  copy[0] = hands[0];
  copy[1] = hands[1];
  hands[0].rebooted = false;
  hands[1].rebooted = false;
  portEXIT_CRITICAL(&dataMux);

  unsigned long now = millis();
  uint8_t st[2];

  // Repeat the startup message once, in case the Bela missed the first one
  if (!repeatedBootInfo && now > 3000) {
    repeatedBootInfo = true;
    Serial.printf("INFO,Receiver running (last reset reason: %s)\n", resetReasonName(bootReason));
  }

  for (int i = 0; i < 2; i++) {
    bool connected = copy[i].everSeen && (now - copy[i].lastSeen < TIMEOUT_MS);
    st[i] = connected ? copy[i].data.setupState : ST_OFFLINE;
    const GloveData &d = copy[i].data;

    // ---- events ----
    if (copy[i].rebooted) lastEventSeq[i] = 0;

    if (connected && !seenInLoop[i]) {
      seenInLoop[i] = true;
      if (d.sequence > 100) lastEventSeq[i] = d.eventSeq;   // don't replay an old event
    }

    if (connected && d.eventSeq != lastEventSeq[i]) {
      const char *name = eventName(d.eventCode);
      if (name) Serial.printf("EVENT,%s,%s\n", handName[i], name);
      lastEventSeq[i] = d.eventSeq;
    }

    // ---- status ----
    uint8_t step = connected ? d.setupStep : 0;
    if (st[i] != lastReportedState[i] || step != lastReportedStep[i] ||
        now - lastStatusMs[i] >= STATUS_INTERVAL_MS) {
      Serial.printf("STATUS,%s,%s,%d\n", handName[i], stateName(st[i]), step);
      lastReportedState[i] = st[i];
      lastReportedStep[i] = step;
      lastStatusMs[i] = now;
    }
  }

  // ---- control broadcast to the gloves ----
  if (now - lastControl >= CONTROL_INTERVAL_MS) {
    lastControl = now;
    ControlData c;
    c.magic = CTRL_MAGIC;
    c.version = CTRL_VERSION;
    c.turn = computeTurn(st);
    c.handState[0] = st[0];
    c.handState[1] = st[1];
    c.session = session;
    c.cmdSeq[0] = cmdSeq[0];
    c.cmdSeq[1] = cmdSeq[1];
    c.cmdType[0] = cmdType[0];
    c.cmdType[1] = cmdType[1];
    esp_now_send(broadcastAddress, (const uint8_t *)&c, sizeof(c));
  }

  // ---- playing data ----
  if (now - lastOutput >= OUTPUT_INTERVAL_MS) {
    lastOutput = now;
    printHand(0, copy[0].data, st[0] == ST_READY);
    printHand(1, copy[1].data, st[1] == ST_READY);
  }

#if RX_DEBUG
  static unsigned long lastDebug = 0;
  if (now - lastDebug >= 1000) {
    lastDebug = now;
    Serial.printf("# L %s | R %s | turn %d | rejected %lu\n",
                  stateName(st[0]), stateName(st[1]), computeTurn(st),
                  (unsigned long)rxRejected);
  }
#endif

  delay(1);
}