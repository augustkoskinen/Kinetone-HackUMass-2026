// ==========================================================
// GESTURAL MIDI GLOVES — BELA RECEIVER
//
// Reads lines from the receiver ESP32 over USB serial:
//   NAME,fingers,roll,motion
//   fingers: index,middle,ring,pinky (1 = touching)
//   roll:    -180..180 (0 = palm-down zone, + = clockwise)
//   motion:  0..100 (not used yet)
//   STATUS,LEFT|RIGHT,state,step
//   EVENT,LEFT|RIGHT,name
//   INFO,text                         // messages from the receiver
// Splits and validates them in a background task, then maps
// finger patterns to a note and plays it.
// plays setup guide tones and keeps the instrument muted until
// every connected glove has finished setup.
//
// INSTRUMENTS: a suite of wind instrument voices, each with its own
// timbre, breath noise, vibrato, attack and octave. Switch them live
// from the Bela IDE console (see printCommandHelp below).
//
// Console commands, e.g.
//   echo setup > /tmp/glove          (re-zero both gloves)
//   echo full right > /tmp/glove     (full calibration, right glove)
//   echo status > /tmp/glove
//   echo inst clarinet > /tmp/glove  (change instrument)
//   echo octave up > /tmp/glove      (shift the instrument an octave)
// ==========================================================

#include <Bela.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <atomic>
#include <chrono>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>   // hold the ESP32's reset lines still
#include <sys/stat.h>    // check whether the receiver is still plugged in
#include <map>
#include <iostream>

// ==========================================
// CONFIGURATION
// ==========================================

// Serial devices to try, in order (the receiver ESP32 is usually ttyUSB0)
const char *kSerialDevices[] = { "/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1" };
const speed_t kBaud = B115200;

// Hand marked disconnected if no line arrives for this long
const double kStaleSeconds = 1.0;

// how the serial port reacts to silence
const double kSilenceWarnSeconds        = 2.0;    // print a warning
const double kSilenceForceReopenSeconds = 20.0;   // reopen even if still plugged in

// glove counted as offline if no STATUS line arrives for this long
const double kStatusStaleSeconds = 3.0;

// setup guide tones
const float kGuideVolume     = 0.25f;
const float kPromptRepeatSec = 6.0f;     // step prompt repeats this often
const float kLeftGuideFreq   = 440.0f;   // left glove tones (lower)
const float kRightGuideFreq  = 880.0f;   // right glove tones (higher)

// file the console commands are written to
const char *kCommandFile = "/tmp/glove";

// ==========================================
// SHARED HAND STATE (written by serial task, read by render)
// ==========================================

enum { kLeft = 0, kRight = 1 };

// setup states and events (match the words the receiver sends)
enum { kStateOffline = 0, kStateWaiting, kStatePrompt, kStateMeasuring, kStateReady };
enum { kEvNone = 0, kEvFullMode, kEvCapture, kEvStepDone, kEvRedo,
       kEvRestart, kEvHandReady, kEvResumed };

struct HandState {
        std::atomic<bool>    connected{false};
        std::atomic<uint8_t> fingers{0};   // bit3 index, bit2 middle, bit1 ring, bit0 pinky
        std::atomic<int>     roll{0};      // -180..180, 0 = palm-down zone, + = clockwise
        std::atomic<int>     motion{0};    // 0..100
        std::atomic<int>     setupState{kStateOffline};
        std::atomic<int>     setupStep{0};
};

HandState gHands[2];

const unsigned kEventQueueSize = 32;
uint8_t gEventQueue[kEventQueueSize];
std::atomic<unsigned> gEventHead{0}, gEventTail{0};

void pushEvent(int hand, int ev)
{
        unsigned head = gEventHead.load();
        unsigned next = (head + 1) % kEventQueueSize;
        if (next == gEventTail.load())
                return;   // queue full: drop
        gEventQueue[head] = (uint8_t)((hand << 4) | ev);
        gEventHead.store(next);
}

bool popEvent(int &hand, int &ev)
{
        unsigned tail = gEventTail.load();
        if (tail == gEventHead.load())
                return false;
        uint8_t v = gEventQueue[tail];
        gEventTail.store((tail + 1) % kEventQueueSize);
        hand = v >> 4;
        ev = v & 0x0F;
        return true;
}

// ==========================================
// INSTRUMENTS
// ==========================================
// Each voice is built from a harmonic recipe at startup (baked into
// wavetables), plus live breath noise, vibrato, envelope and a
// "bloom" filter that makes brass brighten as the note swells.
//
// octave: shift applied to the whole finger mapping for this instrument
//         (+1 = an octave up, -2 = two octaves down)
// To tweak a sound, edit its row below. To add an instrument, add a row.

struct Instrument {
        const char *name;          // console name (lowercase, no spaces)
        const char *alias;         // second accepted name ("" for none)
        const char *display;       // printed name
        int   octave;
        float harm[16];            // harmonic amplitudes, 1st..16th
        float drive;               // saturation baked into the tone (0 = clean)
        float asym;                // reed asymmetry (adds even harmonics)
        float breath;              // breath noise amount
        float breathCutoffHz;      // breath noise brightness
        float vibRateHz;           // vibrato speed
        float vibDepth;            // vibrato depth (0.005 = about 9 cents)
        float attackMs;
        float releaseMs;
        float glideMs;             // slide time between notes
        float bloom;               // 0 = steady brightness, 1 = dark-to-bright with loudness
        float gain;                // level match between instruments
};

const Instrument kInstruments[] = {
//  name        alias         display          oct  harmonics 1..16                                                           drive asym  breath cutoff vibHz vibDep  att  rel  glide bloom gain
  { "flute",    "",           "Flute",          +1, {1.00f,.40f,.12f,.06f,.03f,.015f},                                       0.0f, 0.00f, .10f, 3500,  5.0f, .005f,  60,  90,  25, 0.0f, 0.90f },
  { "piccolo",  "",           "Piccolo",        +2, {1.00f,.25f,.06f,.02f},                                                  0.0f, 0.00f, .08f, 6000,  5.5f, .004f,  40,  70,  20, 0.0f, 0.70f },
  { "clarinet", "",           "Clarinet",        0, {1.00f,.04f,.75f,.03f,.50f,.02f,.32f,.02f,.20f,.01f,.12f,.01f,.07f},     1.2f, 0.00f, .025f,2500,  0.0f, .000f,  35,  70,  20, 0.0f, 0.80f },
  { "oboe",     "",           "Oboe",            0, {.55f,1.00f,.95f,.75f,.60f,.45f,.32f,.22f,.15f,.10f,.07f,.05f},         1.5f, 0.10f, .02f, 2500,  5.5f, .004f,  30,  60,  15, 0.0f, 0.95f },
  { "bassoon",  "",           "Bassoon",        -1, {.45f,1.00f,.85f,.65f,.50f,.35f,.22f,.15f,.10f,.07f,.05f},              1.4f, 0.12f, .02f, 1800,  5.0f, .003f,  45,  80,  25, 0.0f, 1.10f },
  { "altosax",  "alto",       "Alto Sax",        0, {1.00f,.75f,.50f,.40f,.25f,.15f,.10f,.06f},                             1.5f, 0.15f, .04f, 3000,  5.5f, .005f,  40,  80,  20, 0.3f, 0.80f },
  { "tenorsax", "tenor",      "Tenor Sax",      -1, {1.00f,.85f,.60f,.45f,.32f,.22f,.14f,.09f,.05f},                        1.6f, 0.15f, .05f, 2500,  5.0f, .005f,  45,  90,  25, 0.3f, 0.85f },
  { "barisax",  "bari",       "Baritone Sax",   -2, {1.00f,.90f,.70f,.55f,.40f,.28f,.18f,.12f,.08f,.05f},                   1.8f, 0.18f, .05f, 2000,  4.5f, .004f,  50, 100,  30, 0.3f, 0.90f },
  { "trumpet",  "",           "Trumpet",         0, {1.00f,.60f,.80f,.40f,.50f,.25f,.20f,.15f,.10f,.07f},                   2.2f, 0.00f, .015f,4000,  5.5f, .003f,  30,  60,  15, 0.7f, 0.70f },
  { "horn",     "frenchhorn", "French Horn",    -1, {1.00f,.50f,.25f,.12f,.06f,.03f},                                       1.2f, 0.00f, .01f, 1500,  4.5f, .002f,  70, 120,  30, 0.5f, 0.85f },
  { "trombone", "",           "Trombone",       -1, {1.00f,.80f,.60f,.50f,.40f,.30f,.20f,.12f,.08f,.05f},                   1.8f, 0.00f, .01f, 2000,  0.0f, .000f,  50, 100,  40, 0.7f, 0.85f },
  { "tuba",     "",           "Tuba",           -2, {1.00f,.60f,.30f,.15f,.07f,.03f},                                       1.3f, 0.00f, .01f,  800,  4.0f, .002f,  80, 150,  40, 0.4f, 0.95f },
  { "electricguitar", "guitar", "Electric Guitar", 0, {1.00f,.85f,.70f,.55f,.45f,.35f,.28f,.22f,.17f,.13f,.10f,.08f,.06f,.04f,.03f,.02f}, 1.8f, 0.25f, .015f, 5000, 0.0f, .000f,  5, 180,  15, 0.8f, 0.85f },
  { "chiptune", "8bit", "Chiptune", 0, {1.00f,.00f,.75f,.00f,.55f,.00f,.40f,.00f,.30f,.00f,.22f,.00f,.15f,.00f,.10f,.00f}, 1.5f, 0.00f, .000f, 8000, 0.0f, .000f,   2,  90,  10, 0.0f, 0.80f },
};
const int kNumInstruments = sizeof(kInstruments) / sizeof(kInstruments[0]);
const int kDefaultInstrument = 5;   // Alto Sax: same register as the original sax voice

const float kMasterLevel = 0.22f;   // overall instrument volume
const int   kOctaveShiftMin = -2;   // limits for the "octave" console command
const int   kOctaveShiftMax = 2;

// Set by console commands (serial task), read by render
std::atomic<int> gInstrumentRequest{kDefaultInstrument};
std::atomic<int> gOctaveShift{0};

// Find an instrument by number (1..N), name, alias or unique prefix. -1 if none.
int findInstrument(const char *s)
{
        char *end;
        long num = strtol(s, &end, 10);
        if (end != s && *end == '\0')
                return (num >= 1 && num <= kNumInstruments) ? (int)num - 1 : -1;

        for (int i = 0; i < kNumInstruments; i++)
                if (strcmp(s, kInstruments[i].name) == 0 || strcmp(s, kInstruments[i].alias) == 0)
                        return i;

        int found = -1;
        size_t len = strlen(s);
        for (int i = 0; i < kNumInstruments; i++) {
                if (strncmp(s, kInstruments[i].name, len) == 0) {
                        if (found >= 0)
                                return -1;   // ambiguous
                        found = i;
                }
        }
        return found;
}

void printInstrumentList()
{
        int cur = gInstrumentRequest.load();
        printf("Instruments (octave shown is relative to the finger chart):\n");
        for (int i = 0; i < kNumInstruments; i++)
                printf("  %s %2d. %-13s %-9s octave %+d\n", i == cur ? "*" : " ", i + 1,
                       kInstruments[i].display, kInstruments[i].name, kInstruments[i].octave);
}

void printCurrentInstrument()
{
        const Instrument &in = kInstruments[gInstrumentRequest.load()];
        int shift = gOctaveShift.load();
        printf("Instrument: %s (octave %+d", in.display, in.octave);
        if (shift != 0)
                printf(", shifted %+d = %+d", shift, in.octave + shift);
        printf(")\n");
}

// ---- wavetables (built once in setup) ----
// One table per instrument per playing octave, each limited to the
// harmonics that stay below the Nyquist limit at that octave (no aliasing).
const int kTableSize = 2048;
const int kTableOctMin = -4;
const int kTableOctMax = 4;
const int kTableLevels = kTableOctMax - kTableOctMin + 1;
const int kMaxHarmonics = 40;

float gTables[kNumInstruments][kTableLevels][kTableSize + 1];

void buildWavetables(float sampleRate, float highestNoteHz)
{
        static float raw[kTableSize];
        static float sinTab[kTableSize], cosTab[kTableSize];
        for (int i = 0; i < kTableSize; i++) {
                sinTab[i] = sinf(2.0f * (float)M_PI * i / kTableSize);
                cosTab[i] = cosf(2.0f * (float)M_PI * i / kTableSize);
        }

        for (int n = 0; n < kNumInstruments; n++) {
                const Instrument &in = kInstruments[n];

                // 1. Sum the harmonic recipe
                float peak = 0.0f;
                for (int i = 0; i < kTableSize; i++) {
                        float s = 0.0f;
                        for (int k = 1; k <= 16; k++)
                                if (in.harm[k - 1] != 0.0f)
                                        s += in.harm[k - 1] * sinTab[(k * i) % kTableSize];
                        raw[i] = s;
                        peak = fmaxf(peak, fabsf(s));
                }

                // 2. Shape it: saturation and reed asymmetry
                float mean = 0.0f;
                for (int i = 0; i < kTableSize; i++) {
                        float x = raw[i] / peak;
                        if (in.drive > 0.0f)
                                x = tanhf(x * in.drive) / tanhf(in.drive);
                        x += in.asym * x * x;
                        raw[i] = x;
                        mean += x;
                }
                mean /= kTableSize;

                // 3. Measure its harmonics
                float a[kMaxHarmonics + 1], b[kMaxHarmonics + 1];
                for (int k = 1; k <= kMaxHarmonics; k++) {
                        a[k] = b[k] = 0.0f;
                        for (int i = 0; i < kTableSize; i++) {
                                int idx = (k * i) % kTableSize;
                                a[k] += (raw[i] - mean) * sinTab[idx];
                                b[k] += (raw[i] - mean) * cosTab[idx];
                        }
                }

                // 4. Rebuild one band-limited table per octave
                for (int lvl = 0; lvl < kTableLevels; lvl++) {
                        int oct = lvl + kTableOctMin;
                        float maxFreq = highestNoteHz * powf(2.0f, (float)oct) * 1.07f;   // + bend + vibrato headroom
                        int maxH = (int)(0.45f * sampleRate / maxFreq);
                        if (maxH < 1) maxH = 1;
                        if (maxH > kMaxHarmonics) maxH = kMaxHarmonics;

                        float *t = gTables[n][lvl];
                        float tPeak = 0.0f;
                        for (int i = 0; i < kTableSize; i++) {
                                float s = 0.0f;
                                for (int k = 1; k <= maxH; k++) {
                                        int idx = (k * i) % kTableSize;
                                        s += a[k] * sinTab[idx] + b[k] * cosTab[idx];
                                }
                                t[i] = s;
                                tPeak = fmaxf(tPeak, fabsf(s));
                        }
                        for (int i = 0; i < kTableSize; i++)
                                t[i] /= (tPeak > 0.0f ? tPeak : 1.0f);
                        t[kTableSize] = t[0];   // guard point for interpolation
                }
        }
}

// ==========================================
// SERIAL PORT
// ==========================================

using Clock = std::chrono::steady_clock;

const char *gOpenDevice = nullptr;   // which device is open

int openSerial()
{
    for (const char *dev : kSerialDevices) {
        int fd = open(dev, O_RDWR | O_NOCTTY);   // read AND write (commands to the receiver)
        if (fd < 0)
            continue;

        termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            close(fd);
            continue;
        }

        cfmakeraw(&tty);
        cfsetispeed(&tty, kBaud);
        cfsetospeed(&tty, kBaud);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~HUPCL;   // don't toggle the control lines on close (that resets the ESP32)
        tty.c_cc[VMIN]  = 0;     // return whatever is available...
        tty.c_cc[VTIME] = 1;     // ...or after 100 ms

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            close(fd);
            continue;
        }

        // release DTR and RTS together so the ESP32's auto-reset circuit leaves it running
        int lines = TIOCM_DTR | TIOCM_RTS;
        ioctl(fd, TIOCMBIC, &lines);

        tcflush(fd, TCIFLUSH);
        gOpenDevice = dev;
        printf("Serial: opened %s\n", dev);
        return fd;
    }
    return -1;
}

// is the receiver still plugged in?
bool openDeviceStillThere()
{
        struct stat st;
        return gOpenDevice != nullptr && stat(gOpenDevice, &st) == 0;
}

// ==========================================
// LINE PARSING
// ==========================================

std::vector<std::string> split(const std::string &s, char sep)
{
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        size_t pos = s.find(sep, start);
        out.push_back(s.substr(start, pos - start));
        if (pos == std::string::npos)
            break;
        start = pos + 1;
    }
    return out;
}

bool parseFingers(const std::string &f, uint8_t &out)
{
        if (f.size() != 4)
                return false;
        out = 0;
        for (int i = 0; i < 4; i++) {
            if (f[i] == '1')
                out |= (1 << (3 - i));
            else if (f[i] != '0')
                return false;
        }
        return true;
}

bool parseFloat(const std::string &s, float &out)
{
    char *end;
    out = strtof(s.c_str(), &end);
    return end != s.c_str() && *end == '\0';
}

int handFromName(const std::string &s)
{
        if (s == "LEFT")  return kLeft;
        if (s == "RIGHT") return kRight;
        return -1;
}

int stateFromName(const std::string &s)
{
        if (s == "OFFLINE")   return kStateOffline;
        if (s == "WAITING")   return kStateWaiting;
        if (s == "PROMPT")    return kStatePrompt;
        if (s == "MEASURING") return kStateMeasuring;
        if (s == "READY")     return kStateReady;
        return -1;
}

// readable state names for the "status" command
const char *stateToName(int s)
{
        switch (s) {
        case kStateWaiting:   return "WAITING for its turn";
        case kStatePrompt:    return "waiting for index tap";
        case kStateMeasuring: return "measuring";
        case kStateReady:     return "READY";
        default:              return "OFFLINE";
        }
}

int eventFromName(const std::string &s)
{
        if (s == "FULL_MODE")  return kEvFullMode;
        if (s == "CAPTURE")    return kEvCapture;
        if (s == "STEP_DONE")  return kEvStepDone;
        if (s == "REDO")       return kEvRedo;
        if (s == "RESTART")    return kEvRestart;
        if (s == "HAND_READY") return kEvHandReady;
        if (s == "RESUMED")    return kEvResumed;
        return -1;
}

Clock::time_point gLastSeen[2];
Clock::time_point gLastStatus[2];

void processLine(const std::string &line, Clock::time_point now)
{
    // Skip separators, diagnostics, blank lines
    if (line.empty() || line[0] == '#' || line == "---")
        return;

    std::vector<std::string> f = split(line, ',');

    // message from the receiver, e.g. INFO,Receiver started (reset reason: BROWNOUT)
    if (f[0] == "INFO") {
        printf("Receiver: %s\n", line.c_str() + 5);
        return;
    }

    // setup status, e.g. STATUS,LEFT,PROMPT,2
    if (f[0] == "STATUS") {
        if (f.size() != 4) return;
        int hand = handFromName(f[1]);
        int st = stateFromName(f[2]);
        if (hand < 0 || st < 0) return;
        char *end;
        long step = strtol(f[3].c_str(), &end, 10);
        if (end == f[3].c_str() || *end != '\0') return;
        gHands[hand].setupStep = (int)step;
        gHands[hand].setupState = st;
        gLastStatus[hand] = now;
        return;
    }

    // setup event, e.g. EVENT,RIGHT,STEP_DONE
    if (f[0] == "EVENT") {
        if (f.size() != 3) return;
        int hand = handFromName(f[1]);
        int ev = eventFromName(f[2]);
        if (hand < 0 || ev < 0) return;
        printf("Setup: %s %s\n", f[1].c_str(), f[2].c_str());
        pushEvent(hand, ev);
        return;
    }

    if (f.size() != 4)
        return;   // boot text, garbage or half-received line

    int hand;
    if (f[0] == "LEFT")
        hand = kLeft;
    else if (f[0] == "RIGHT")
        hand = kRight;
    else
        return;

    uint8_t fingers;
    if (!parseFingers(f[1], fingers))
        return;

    char *end;
    long roll = strtol(f[2].c_str(), &end, 10);
    if (end == f[2].c_str() || *end != '\0')
            return;
    long motion = strtol(f[3].c_str(), &end, 10);
    if (end == f[3].c_str() || *end != '\0')
        return;
    if (roll < -180 || roll > 180 || motion < 0 || motion > 100)
        return;

    // Line is valid: publish it
    HandState &h = gHands[hand];
    h.fingers   = fingers;
    h.roll      = (int)roll;
    h.motion    = (int)motion;
    h.connected = true;

    gLastSeen[hand] = now;
}

// ==========================================
// CONSOLE COMMANDS
// ==========================================

void printCommandHelp()
{
        printf("Glove commands (type in the Bela IDE console):\n");
        printf("  echo setup > /tmp/glove          re-zero both gloves\n");
        printf("  echo setup left > /tmp/glove     re-zero one glove (left/right)\n");
        printf("  echo full > /tmp/glove           full calibration, both gloves\n");
        printf("  echo full right > /tmp/glove     full calibration, one glove\n");
        printf("  echo status > /tmp/glove         show glove states and instrument\n");
        printf("Instrument commands:\n");
        printf("  echo inst > /tmp/glove           list instruments\n");
        printf("  echo inst oboe > /tmp/glove      switch by name (or number, e.g. inst 3)\n");
        printf("  echo inst next > /tmp/glove      next instrument (or: inst prev)\n");
        printf("  echo octave up > /tmp/glove      shift up an octave (down, 0, or -2..+2)\n");
}

// "inst ..." command
void instrumentCommand(int words, const char *arg)
{
        if (words < 2 || strcmp(arg, "list") == 0) {
                printInstrumentList();
                return;
        }

        int cur = gInstrumentRequest.load();
        int idx;
        if (strcmp(arg, "next") == 0)
                idx = (cur + 1) % kNumInstruments;
        else if (strcmp(arg, "prev") == 0)
                idx = (cur + kNumInstruments - 1) % kNumInstruments;
        else
                idx = findInstrument(arg);

        if (idx < 0) {
                printf("Unknown instrument '%s'\n", arg);
                printInstrumentList();
                return;
        }

        gInstrumentRequest = idx;
        printCurrentInstrument();
}

// "octave ..." command
void octaveCommand(int words, const char *arg)
{
        int shift = gOctaveShift.load();

        if (words >= 2) {
                if (strcmp(arg, "up") == 0)
                        shift++;
                else if (strcmp(arg, "down") == 0)
                        shift--;
                else if (strcmp(arg, "reset") == 0)
                        shift = 0;
                else {
                        char *end;
                        long v = strtol(arg, &end, 10);
                        if (end == arg || *end != '\0') {
                                printf("Use: octave up | down | reset | -2..+2\n");
                                return;
                        }
                        shift = (int)v;
                }
        }

        if (shift < kOctaveShiftMin) shift = kOctaveShiftMin;
        if (shift > kOctaveShiftMax) shift = kOctaveShiftMax;
        gOctaveShift = shift;
        printCurrentInstrument();
}

// Checks for a command file, runs it, deletes it
void checkCommandFile(int fd)
{
        FILE *fp = fopen(kCommandFile, "r");
        if (!fp)
                return;

        char line[64] = {0};
        if (!fgets(line, sizeof(line), fp))
                line[0] = '\0';
        fclose(fp);
        unlink(kCommandFile);

        for (char *p = line; *p; p++)
                *p = (char)tolower((unsigned char)*p);

        char w1[16] = {0}, w2[16] = {0};
        int words = sscanf(line, "%15s %15s", w1, w2);
        if (words < 1) {
                printCommandHelp();
                return;
        }

        if (strcmp(w1, "status") == 0) {
                printf("Left glove:  %s\n", stateToName(gHands[kLeft].setupState));
                printf("Right glove: %s\n", stateToName(gHands[kRight].setupState));
                printCurrentInstrument();
                return;
        }

        if (strcmp(w1, "inst") == 0 || strcmp(w1, "instrument") == 0) {
                instrumentCommand(words, w2);
                return;
        }

        if (strcmp(w1, "octave") == 0 || strcmp(w1, "oct") == 0) {
                octaveCommand(words, w2);
                return;
        }

        if (strcmp(w1, "help") == 0) {
                printCommandHelp();
                return;
        }

        const char *type;
        if (strcmp(w1, "setup") == 0 || strcmp(w1, "rezero") == 0)
                type = "SETUP";
        else if (strcmp(w1, "full") == 0 || strcmp(w1, "calibrate") == 0)
                type = "FULL";
        else {
                printf("Unknown command '%s'\n", w1);
                printCommandHelp();
                return;
        }

        const char *hand;
        if (words < 2 || strcmp(w2, "both") == 0)
                hand = "BOTH";
        else if (strcmp(w2, "left") == 0)
                hand = "LEFT";
        else if (strcmp(w2, "right") == 0)
                hand = "RIGHT";
        else {
                printf("Unknown glove '%s' (use left, right or both)\n", w2);
                return;
        }

        if (fd < 0) {
                printf("Can't send command: receiver not connected\n");
                return;
        }

        char out[48];
        snprintf(out, sizeof(out), "CMD,%s,%s\n", type, hand);
        if (write(fd, out, strlen(out)) < 0)
                printf("Can't send command: serial write failed\n");
        else
                printf("Sent to receiver: %s", out);
}

// ==========================================
// BACKGROUND SERIAL TASK
// ==========================================

void serialTask(void *)
{
    int fd = -1;
    std::string lineBuf;
    char buf[256];
    Clock::time_point lastData = Clock::now();
    bool warnedSilent = false;

    unlink(kCommandFile);   // ignore anything left over from before
    printCommandHelp();
    printCurrentInstrument();

    while (!Bela_stopRequested()) {

        checkCommandFile(fd);

        if (fd < 0) {
            gHands[kLeft].connected = false;
            gHands[kRight].connected = false;
            gHands[kLeft].setupState = kStateOffline;
            gHands[kRight].setupState = kStateOffline;

            fd = openSerial();
            if (fd < 0) {
                usleep(500000);
                continue;
            }
            lineBuf.clear();
            lastData = Clock::now();
            warnedSilent = false;
        }

        ssize_t n = read(fd, buf, sizeof(buf));
        Clock::time_point now = Clock::now();

        if (n < 0) {
            printf("Serial: read error, reopening\n");
            close(fd);
            fd = -1;
            continue;
        }

        if (n > 0) {
            lastData = now;
            if (warnedSilent) {
                printf("Serial: receiver data is back\n");
                warnedSilent = false;
            }
        }

        for (ssize_t i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\n') {
                if (!lineBuf.empty() && lineBuf.back() == '\r')
                    lineBuf.pop_back();
                processLine(lineBuf, now);
                lineBuf.clear();
            } else if (lineBuf.size() < 256) {
                lineBuf += c;
            } else {
                lineBuf.clear();   // runaway line: discard
            }
        }

        // gentle handling of silence. A receiver that is briefly quiet
        // (e.g. restarting) is left alone; reopening the port can reset it again.
        double silent = std::chrono::duration<double>(now - lastData).count();

        if (silent > kSilenceWarnSeconds && !warnedSilent) {
            printf("Serial: receiver silent - waiting for it (port kept open)\n");
            warnedSilent = true;
        }

        if (silent > kSilenceWarnSeconds && !openDeviceStillThere()) {
            printf("Serial: receiver unplugged, reconnecting\n");
            close(fd);
            fd = -1;
            continue;
        }

        if (silent > kSilenceForceReopenSeconds) {
            printf("Serial: no data for %.0f s, reopening\n", silent);
            close(fd);
            fd = -1;
            continue;
        }

        for (int hand = 0; hand < 2; hand++) {
            // A hand whose lines have stopped arriving counts as disconnected
            if (gHands[hand].connected &&
                std::chrono::duration<double>(now - gLastSeen[hand]).count() > kStaleSeconds)
                    gHands[hand].connected = false;

            // no setup status for a while = glove offline
            if (gHands[hand].setupState != kStateOffline &&
                std::chrono::duration<double>(now - gLastStatus[hand]).count() > kStatusStaleSeconds)
                    gHands[hand].setupState = kStateOffline;
        }
    }

    if (fd >= 0)
        close(fd);
}

AuxiliaryTask gSerialTask;

// ==========================================
// SETUP GUIDE TONES (render thread only)
// ==========================================

struct GuideNote { float freq; int on; int off; };   // lengths in samples

const int kGuideQueueSize = 64;
GuideNote gGuide[kGuideQueueSize];
int gGuideRead = 0, gGuideWrite = 0;
GuideNote gGuideCur;
bool gGuidePlaying = false;
int gGuidePos = 0;
float gGuidePhase = 0.0f;
float gSampleRate = 44100.0f;

void guideAdd(float freq, float onMs, float offMs)
{
        int next = (gGuideWrite + 1) % kGuideQueueSize;
        if (next == gGuideRead)
                return;
        gGuide[gGuideWrite] = { freq,
                                (int)(onMs * 0.001f * gSampleRate),
                                (int)(offMs * 0.001f * gSampleRate) };
        gGuideWrite = next;
}

float guideSample()
{
        if (!gGuidePlaying) {
                if (gGuideRead == gGuideWrite)
                        return 0.0f;
                gGuideCur = gGuide[gGuideRead];
                gGuideRead = (gGuideRead + 1) % kGuideQueueSize;
                gGuidePos = 0;
                gGuidePhase = 0.0f;
                gGuidePlaying = true;
        }

        float s = 0.0f;
        if (gGuidePos < gGuideCur.on) {
                int fade = (int)(0.005f * gSampleRate);   // 5 ms fades, no clicks
                float env = 1.0f;
                if (gGuidePos < fade)
                        env = (float)gGuidePos / fade;
                else if (gGuideCur.on - gGuidePos < fade)
                        env = (float)(gGuideCur.on - gGuidePos) / fade;
                s = env * sinf(gGuidePhase) * kGuideVolume;
                gGuidePhase += 2.0f * (float)M_PI * gGuideCur.freq / gSampleRate;
                if (gGuidePhase > 2.0f * (float)M_PI)
                        gGuidePhase -= 2.0f * (float)M_PI;
        }

        gGuidePos++;
        if (gGuidePos >= gGuideCur.on + gGuideCur.off)
                gGuidePlaying = false;
        return s;
}

float handBaseFreq(int hand) { return hand == kLeft ? kLeftGuideFreq : kRightGuideFreq; }

// Step N = N beeps
void playPrompt(int hand, int step)
{
        float b = handBaseFreq(hand);
        for (int i = 0; i < step; i++)
                guideAdd(b, 120, 150);
}

void playEvent(int hand, int ev)
{
        float b = handBaseFreq(hand);
        switch (ev) {
        case kEvFullMode:  guideAdd(b, 60, 60); guideAdd(b, 60, 60); guideAdd(b * 2.0f, 250, 300); break;
        case kEvCapture:   guideAdd(b * 1.5f, 40, 0); break;                                         // tick: hold still
        case kEvStepDone:  guideAdd(b, 100, 20); guideAdd(b * 1.5f, 180, 250); break;                // rising chime
        case kEvRedo:      guideAdd(180.0f, 350, 300); break;                                         // low buzz
        case kEvRestart:   guideAdd(180.0f, 250, 120); guideAdd(150.0f, 400, 400); break;             // double buzz
        case kEvHandReady: guideAdd(b, 100, 20); guideAdd(b * 1.26f, 100, 20); guideAdd(b * 1.5f, 300, 400); break;
        case kEvResumed:   guideAdd(b * 1.5f, 80, 40); guideAdd(b, 80, 40); guideAdd(b * 1.5f, 80, 300); break;
        }
}

void playAllReady()
{
        guideAdd(523.25f, 120, 20);
        guideAdd(659.25f, 120, 20);
        guideAdd(783.99f, 120, 20);
        guideAdd(1046.5f, 450, 200);
}

bool gPlayable = false;          // instrument allowed to sound
float gInstrumentGain = 0.0f;
int gPromptHand = -1, gPromptStep = -1;
uint64_t gNextPromptFrame = 0;

void updateSetupGuide(BelaContext *context)
{
        uint64_t now = context->audioFramesElapsed;

        int hand, ev;
        while (popEvent(hand, ev))
                playEvent(hand, ev);

        int ls = gHands[kLeft].setupState;
        int rs = gHands[kRight].setupState;

        bool anyConnected = (ls != kStateOffline) || (rs != kStateOffline);
        bool allReady = anyConnected &&
                        (ls == kStateOffline || ls == kStateReady) &&
                        (rs == kStateOffline || rs == kStateReady);

        if (allReady && !gPlayable)
                playAllReady();
        gPlayable = allReady;

        // Prompt the glove that is waiting for an index tap
        int ph = -1;
        if (ls == kStatePrompt) ph = kLeft;
        else if (rs == kStatePrompt) ph = kRight;

        if (ph >= 0) {
                int step = gHands[ph].setupStep;
                if (ph != gPromptHand || step != gPromptStep) {
                        gPromptHand = ph;
                        gPromptStep = step;
                        gNextPromptFrame = now + (uint64_t)(0.4f * gSampleRate);   // let chimes finish first
                }
                if (now >= gNextPromptFrame) {
                        playPrompt(ph, step);
                        gNextPromptFrame = now + (uint64_t)(kPromptRepeatSec * gSampleRate);
                }
        } else {
                gPromptHand = -1;
                gPromptStep = -1;
        }
}

// ==========================================
// AUDIO
// ==========================================

// note mappings: every note from C1 to B7 (A4 = 440 Hz), plus REST.
// Names use sharps: C, C#, D, D#, E, F, F#, G, G#, A, A#, B + octave number.
std::map<std::string, float> buildNoteTable()
{
    static const char *names[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
    std::map<std::string, float> table;
    table["REST"] = 0.0f;
    for (int oct = 1; oct <= 7; oct++)
        for (int i = 0; i < 12; i++) {
            int midi = (oct + 1) * 12 + i;
            table[std::string(names[i]) + std::to_string(oct)] = 440.0f * powf(2.0f, (midi - 69) / 12.0f);
        }
    return table;
}
const std::map<std::string, float> notes = buildNoteTable();

// instrument mappings
// Written at "chart pitch": each instrument then plays it shifted by its
// own octave (see the Instrument table), so one chart works for all of them.
const std::map<std::string, std::string> liamMap = {
    {"0000000", "REST"},
    {"0100010", "A#5"},
    {"1000010", "A5"},
    {"1001010", "G#5"},
    {"1100010", "G5"},
    {"0110001", "F#5"},
    {"0110000", "F5"},
    {"0110100", "E5"},
    {"0111000", "D#5"},
    {"0111100", "D5"},
    {"0111111", "C#5"},
    {"0111110", "C5"},
    {"0010000", "B4"},
    {"0100000", "A#4"},
    {"1000000", "A4"},
    {"1001000", "G#4"},
    {"1100000", "G4"},
    {"1110001", "F#4"},
    {"1110000", "F4"},
    {"1110100", "E4"},
    {"1111000", "D#4"},
    {"1111100", "D4"},
    {"1111111", "C#4"},
    {"1111110", "C4"},
    {"1111011", "B3"},
    {"1111010", "A#3"}
};

const std::map<std::string, std::string> augustMap = {
    
    {"0000000", "REST"},
    
    {"001000", "D5"},
    {"0011010", "C#6"},
    {"0011000", "C6"},
    {"0011100", "B5"},
    {"0100010", "A#5"},
    {"1000010", "A5"},
    {"0001000", "G#5"},
    {"0100000", "G5"},
    {"0110001", "F#5"},
    {"0110000", "F5"},
    {"0110100", "E5"},
    {"0111000", "D#5"},
    {"0111100", "D5"},
    {"0111101", "C#5"},
    {"0111110", "C5"},
    {"0010000", "B4"},
    {"0100000", "A#4"},
    {"1000000", "A4"},
    {"1001000", "G#4"},
    {"1100000", "G4"},
    {"1110001", "F#4"},
    {"1110000", "F4"},
    {"1110100", "E4"},
    {"1111000", "D#4"},
    {"1111100", "D4"},
    {"1111101", "C#4"},
    {"1111110", "C4"}
};

const std::map<std::string, std::string> tenorSaxMapping = {
    {"0000000", "REST"},
    {"0100000", "A#4"},
    {"1000000", "A4"},
    {"1001000", "G#4"},
    {"1100000", "G4"},
    {"1110000", "F4"},
    {"1110100", "E4"},
    {"1111000", "D#4"},
    {"1111100", "D4"},
    {"1111110", "C4"},
    {"1111111", "A#3"}
};

const std::map<std::string, std::string> testMapping = {
    {"0000000", "REST"},
    {"1000000", "C5"},
    {"0100000", "B4"},
    {"0010000", "A4"},
    {"0001000", "G4"},
    {"0000100", "F4"},
    {"0000010", "E4"},
    {"0000001", "D4"}
};

// Phase accumulator
float phase = 0.0f;
float currentFrequency = 0.0f;
float targetFrequency = 0.0f;
float volumeControl = 0.8f;

//setup variables
std::map<std::string, std::string> mapping;
const float kPitchBendSemitones = 1.0f;
const float kBendGlideSec = 0.06f;   // bend glide time; must be longer than the 20 ms update interval
float leftRollPosition = 0;
float rightRollPosition = 0;         // smoothed right-hand roll (-180..180)
float rightRollStage1 = 0;           // first smoothing stage
float secondsElapsed = 0;

// voice state (render thread only)
int      gCurInstrument = kDefaultInstrument;
int      gPendingInstrument = -1;    // instrument waiting to fade in
float    gSwitchGain = 1.0f;         // dips to 0 while switching instruments
float    gEnv = 0.0f;                // note envelope 0..1
float    gWavePhase = 0.0f;          // 0..1 position in the wavetable
float    gVibPhase = 0.0f;
float    gVibRamp = 0.0f;            // vibrato fades in after each new note
float    gToneLP = 0.0f;             // bloom filter state
float    gNoiseLP = 0.0f;            // breath noise filter state
uint32_t gNoiseSeed = 22222;
float    gLastTarget = 0.0f;


//=====================FUNCTIONS=====================
float fingersToFrequency(const std::map<std::string, float> &notes, const std::map<std::string, std::string> &mapping,
        bool leftConnected, uint8_t leftFingers, bool rightConnected, uint8_t rightFingers)
{
        std::string input = "";
    if (!leftConnected && !rightConnected) {
        return notes.at("REST");
    }

        // Left hand: 3 bits (index, middle, ring = bits 3 down to 1)
        if (leftConnected) {
            for (int i = 3; i >= 1; i--) {
                input += ((leftFingers >> i) & 1) ? '1' : '0';
            }
        } else {
            input += "000"; // 3 zeroes if disconnected
        }

        // Right hand: 4 bits (3 down to 0)
        if (rightConnected) {
            for (int i = 3; i >= 0; i--) {
                input += ((rightFingers >> i) & 1) ? '1' : '0';
            }
        } else {
            input += "0000"; // 4 zeroes if disconnected
        }

    return mapping.count(input) ? notes.at(mapping.at(input)) : notes.at("REST");
}

float normalize(float position) {
        float normalized = position;

        while(normalized>180)
                normalized-=360;
        while(normalized<-180)
                normalized+=360;
        return normalized;
}

float leftPositionToOctave(float position) {
        return 1;

        float normalized = normalize(position);

        if(normalized > 45 && normalized <= 135)
                return 0.5;
        if(normalized >= -135 && normalized < -45)
                return 2;
        return 1;
}

float rightPositionToBend(float roll, float semitones) {
        return powf(2.0f, (roll / 180.0f) * semitones / 12.0f);
}

// Highest note in a mapping (used to band-limit the wavetables)
float highestNote(const std::map<std::string, std::string> &m)
{
        float hi = 0.0f;
        for (const auto &p : m)
                if (notes.count(p.second))
                        hi = fmaxf(hi, notes.at(p.second));
        return hi > 0.0f ? hi : 1000.0f;
}

// Fast white noise, -1..1
inline float noiseSample()
{
        gNoiseSeed = gNoiseSeed * 1664525u + 1013904223u;
        return (int32_t)gNoiseSeed * (1.0f / 2147483648.0f);
}

// One sample of the current wind instrument.
// tableFreq: final pitch in Hz. table: wavetable for this octave.
inline float generateSampleWind(const Instrument &in, const float *table, float tableFreq,
                                float lpCoef, float noiseCoef)
{
        // Wavetable oscillator with linear interpolation
        gWavePhase += tableFreq / gSampleRate;
        if (gWavePhase >= 1.0f)
                gWavePhase -= 1.0f;
        float pos = gWavePhase * kTableSize;
        int i = (int)pos;
        float frac = pos - i;
        float tone = table[i] + (table[i + 1] - table[i]) * frac;

        // Bloom: brass and saxes get brighter as the note swells
        if (in.bloom > 0.0f) {
                gToneLP += (tone - gToneLP) * lpCoef;
                tone = gToneLP;
        }

        // Breath noise, a little stronger during the attack ("chiff")
        float noise = 0.0f;
        if (in.breath > 0.0f) {
                gNoiseLP += (noiseSample() - gNoiseLP) * noiseCoef;
                noise = gNoiseLP * in.breath * (1.0f + 3.0f * (1.0f - gEnv)) * 2.0f;
        }

        return (tone + noise) * gEnv;
}

// Legacy voices from the original code (not used, kept for reference)
float generateSampleSin(BelaContext *context, float *phase, float volumeControl, float frequency) {
        float sample = 0.0f;

    if(frequency > 0.0f) {
        float inverseSampleRate = 1.0f / context->audioSampleRate;
        *phase += 2.0f * M_PI * frequency * inverseSampleRate;
        if(*phase > 2.0f * M_PI) {
            *phase -= 2.0f * M_PI;
        }
        sample = sin(*phase) * volumeControl * 0.3f;
    }

    return sample;
}

float generateSampleSax(BelaContext *context, float *phase, float volumeControl, float frequency) {
    float sample = 0.0f;

    if(frequency > 0.0f) {
        float inverseSampleRate = 1.0f / context->audioSampleRate;
        *phase += 2.0f * M_PI * frequency * inverseSampleRate;
        if(*phase > 2.0f * M_PI) {
            *phase -= 2.0f * M_PI;
        }
        float h1 = sin(*phase * 1.0f) * 1.00f;
        float h2 = sin(*phase * 2.0f) * 0.75f;
        float h3 = sin(*phase * 3.0f) * 0.50f;
        float h4 = sin(*phase * 4.0f) * 0.40f;
        float h5 = sin(*phase * 5.0f) * 0.25f;
        float h6 = sin(*phase * 6.0f) * 0.15f;
        float rawSaxWave = h1 + h2 + h3 + h4 + h5 + h6;
        float reedyWave = rawSaxWave + (rawSaxWave * rawSaxWave * 0.15f);
        float saturated = tanh(reedyWave * 1.5f) / tanh(1.5f);
        sample = saturated * 0.2f;
    }

    return sample;
}

float generateSampleTrumpet(BelaContext *context, float *phase, float volumeControl, float frequency) {
    float sample = 0.0f;

    if(frequency > 0.0f) {
        float inverseSampleRate = 1.0f / context->audioSampleRate;
        *phase += 2.0f * M_PI * frequency * inverseSampleRate;
        if(*phase > 2.0f * M_PI) {
            *phase -= 2.0f * M_PI;
        }
        float h1 = sin(*phase * 1.0f) * 1.00f;
        float h2 = sin(*phase * 2.0f) * 0.60f;
        float h3 = sin(*phase * 3.0f) * 0.80f;
        float h4 = sin(*phase * 4.0f) * 0.40f;
        float h5 = sin(*phase * 5.0f) * 0.50f;
        float h6 = sin(*phase * 6.0f) * 0.25f;
        float rawBrassWave = h1 + h2 + h3 + h4 + h5 + h6;
        float drive = 2.2f;
        float saturated = tanh(rawBrassWave * drive) / tanh(drive);
        sample = saturated * 0.2f;
    }

    return sample;
}

//=====================SETUP=====================
bool setup(BelaContext *context, void *userData)
{
    gSampleRate = context->audioSampleRate;   // needed by the guide tones and instruments

    mapping = liamMap;

    // Build every instrument's wavetables, band-limited for the highest note in the chart
    float hi = fmaxf(highestNote(liamMap), fmaxf(highestNote(tenorSaxMapping), highestNote(testMapping)));
    buildWavetables(gSampleRate, hi);

    gSerialTask = Bela_createAuxiliaryTask(serialTask, 50, "glove-serial");
    if (gSerialTask == 0) {
        printf("Error: could not create serial task\n");
        return false;
    }
    Bela_scheduleAuxiliaryTask(gSerialTask);

    rt_printf("Kinetone Initialized! %d instruments loaded\n", kNumInstruments);
    return true;
}

//=====================RENDER=====================
void render(BelaContext *context, void *userData) {
    // setup guide tones, and decide whether the instrument may sound
    updateSetupGuide(context);
    float instrumentTarget = gPlayable ? 1.0f : 0.0f;

    // Read the latest glove state once per audio block
    bool    leftConnected  = gHands[kLeft].connected.load();
    uint8_t leftFingers    = gHands[kLeft].fingers.load();
    bool    rightConnected = gHands[kRight].connected.load();
    uint8_t rightFingers   = gHands[kRight].fingers.load();
    int     rightRoll      = gHands[kRight].roll.load();
    int     leftRoll       = gHands[kLeft].roll.load();

    float rightRollTarget = rightConnected ? (float)rightRoll : 0.0f;

    targetFrequency = fingersToFrequency(notes, mapping, leftConnected, leftFingers, rightConnected, rightFingers);

    // Instrument change requested from the console: fade out, swap, fade in
    int requested = gInstrumentRequest.load();
    if (requested != gCurInstrument && gPendingInstrument < 0)
        gPendingInstrument = requested;

    const Instrument &in = kInstruments[gCurInstrument];
    float sr = context->audioSampleRate;

    // Pick the band-limited table for this instrument's octave
    int oct = in.octave + gOctaveShift.load();
    if (oct < kTableOctMin) oct = kTableOctMin;
    if (oct > kTableOctMax) oct = kTableOctMax;
    const float *table = gTables[gCurInstrument][oct - kTableOctMin];
    float octMul = powf(2.0f, (float)oct);

    // Per-block coefficients
    float bendCoeff   = 1.0f - expf(-1.0f / (kBendGlideSec * sr));
    float attackCoef  = 1.0f - expf(-1.0f / (in.attackMs * 0.001f * sr));
    float releaseCoef = 1.0f - expf(-1.0f / (in.releaseMs * 0.001f * sr));
    float glideCoef   = 1.0f - expf(-1.0f / (in.glideMs * 0.001f * sr));
    float switchStep  = 1.0f / (0.015f * sr);   // 15 ms crossfade when switching
    float vibRampStep = 1.0f / (0.4f * sr);     // vibrato fades in over 0.4 s

    float noiseCoef = 1.0f - expf(-2.0f * (float)M_PI * in.breathCutoffHz / sr);
    float bloomHz = currentFrequency * octMul * (2.0f + 14.0f * (1.0f - in.bloom + in.bloom * gEnv));
    if (bloomHz > 0.45f * sr) bloomHz = 0.45f * sr;
    float lpCoef = 1.0f - expf(-2.0f * (float)M_PI * bloomHz / sr);

    // New note after a rest: start at the right pitch instead of sliding in
    if (targetFrequency > 0.0f && gLastTarget <= 0.0f && gEnv < 0.05f) {
        currentFrequency = targetFrequency;
        gVibRamp = 0.0f;
        gToneLP = 0.0f;
    }
    gLastTarget = targetFrequency;
    float envTarget = targetFrequency > 0.0f ? 1.0f : 0.0f;
    bool switchedThisBlock = false;

    for(unsigned int n = 0; n < context->audioFrames; n++) {
        secondsElapsed = (float)context->audioFramesElapsed / context->audioSampleRate;

        // Glide between notes; during a rest, hold the last pitch while it fades
        if (targetFrequency > 0.0f)
            currentFrequency += (targetFrequency - currentFrequency) * glideCoef;

        // Note envelope
        gEnv += (envTarget - gEnv) * (envTarget > gEnv ? attackCoef : releaseCoef);

        // Instrument switch crossfade
        if (switchedThisBlock) {
            gSwitchGain = 0.0f;   // stay silent until the new instrument's settings load next block
        } else if (gPendingInstrument >= 0) {
            gSwitchGain -= switchStep;
            if (gSwitchGain <= 0.0f) {
                gSwitchGain = 0.0f;
                gCurInstrument = gPendingInstrument;
                gPendingInstrument = -1;
                gToneLP = gNoiseLP = 0.0f;
                switchedThisBlock = true;
            }
        } else if (gSwitchGain < 1.0f) {
            gSwitchGain = fminf(1.0f, gSwitchGain + switchStep);
        }

        // Pitch bend (right hand roll)
        rightRollStage1   += (rightRollTarget - rightRollStage1) * bendCoeff;
        rightRollPosition += (rightRollStage1 - rightRollPosition) * bendCoeff;
        leftRollPosition = leftRoll;

        float bendFactor = rightPositionToBend(rightRollPosition, kPitchBendSemitones);
        float octaveFactor = leftPositionToOctave(leftRollPosition);

        // Vibrato
        float vib = 1.0f;
        if (in.vibDepth > 0.0f) {
            gVibRamp = fminf(1.0f, gVibRamp + vibRampStep);
            gVibPhase += in.vibRateHz / sr;
            if (gVibPhase >= 1.0f) gVibPhase -= 1.0f;
            vib = 1.0f + in.vibDepth * gVibRamp * sinf(2.0f * (float)M_PI * gVibPhase);
        }

        float mutatedFrequency = currentFrequency * octMul * octaveFactor * bendFactor * vib;

        float sample = 0.0f;
        if (mutatedFrequency > 0.0f)
            sample = generateSampleWind(in, table, mutatedFrequency, lpCoef, noiseCoef)
                     * in.gain * kMasterLevel * gSwitchGain;

        // mute the instrument during setup and mix in the guide tones
        gInstrumentGain += (instrumentTarget - gInstrumentGain) * 0.001f;
        sample = sample * gInstrumentGain + guideSample();

        audioWrite(context, n, 0, sample);
        audioWrite(context, n, 1, sample);
    }
}

void cleanup(BelaContext *context, void *userData) {}