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
// Splits and validates them in a background task, then maps
// finger patterns to a note and plays it.
// plays setup guide tones and keeps the instrument muted until
// every connected glove has finished setup.
// ==========================================================

#include <Bela.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <chrono>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
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

// Reopen the serial port if nothing at all arrives for this long
const double kSilenceReopenSeconds = 2.0;

// glove counted as offline if no STATUS line arrives for this long
const double kStatusStaleSeconds = 3.0;

// setup guide tones
const float kGuideVolume     = 0.25f;
const float kPromptRepeatSec = 6.0f;     // step prompt repeats this often
const float kLeftGuideFreq   = 440.0f;   // left glove tones (lower)
const float kRightGuideFreq  = 880.0f;   // right glove tones (higher)

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
// SERIAL PORT
// ==========================================

using Clock = std::chrono::steady_clock;

int openSerial()
{
    for (const char *dev : kSerialDevices) {
        int fd = open(dev, O_RDONLY | O_NOCTTY);
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
        tty.c_cc[VMIN]  = 0;   // return whatever is available...
        tty.c_cc[VTIME] = 1;   // ...or after 100 ms

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            close(fd);
            continue;
        }

        tcflush(fd, TCIFLUSH);
        printf("Serial: opened %s\n", dev);
        return fd;
    }
    return -1;
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
// BACKGROUND SERIAL TASK
// ==========================================

void serialTask(void *)
{
    int fd = -1;
    std::string lineBuf;
    char buf[256];
    Clock::time_point lastData = Clock::now();

    while (!Bela_stopRequested()) {

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
        }

        ssize_t n = read(fd, buf, sizeof(buf));
        Clock::time_point now = Clock::now();

        if (n < 0) {
            printf("Serial: read error, reopening\n");
            close(fd);
            fd = -1;
            continue;
        }

        if (n > 0)
            lastData = now;

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

        // Receiver unplugged or silent: reopen the port
        if (std::chrono::duration<double>(now - lastData).count() > kSilenceReopenSeconds) {
            printf("Serial: no data, reopening\n");
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

// note mappings
const std::map<std::string, float> notes = {
    {"REST", 0.00f},
    {"A#3",  233.08f},
    {"B3",   246.94f},
    {"C4",   261.63f},
    {"C#4",  277.18f},
    {"D4",   293.66f},
    {"D#4",  311.13f},
    {"E4",   329.63f},
    {"F4",   349.23f},
    {"F#4",  369.99f},
    {"G4",   392.00f},
    {"G#4",  415.30f},
    {"A4",   440.00f},
    {"A#4",  466.16f},
    {"B4",   493.88f},
};

// instrument mappings
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
    {"0100000", "A#4"},
    {"0010000", "A4"},
    {"0001000", "G4"},
    {"0000100", "F#4"},
    {"0000010", "F4"},
    {"0000001", "E4"}
};

// Phase accumulator
float phase = 0.0f;
float currentFrequency = 0.0f;
float targetFrequency = 0.0f;
float volumeControl = 0.8f; 

//setup variables
std::map<std::string, std::string> mapping;
const float kPitchBendSemitones = 1.0f;
const float kBendSmoothing = 0.002f;
float leftRollPosition = 0; // [0, 2pi)
float rightRollPosition = 0; // [0, 2pi)
float secondsElapsed = 0;


//=====================FUNCTIONS=====================
float fingersToFrequency(const std::map<std::string, float> &notes, const std::map<std::string, std::string> &mapping,
        bool leftConnected, uint8_t leftFingers, bool rightConnected, uint8_t rightFingers)
{
        std::string input = "";
    if (!leftConnected && !rightConnected) {
        return notes.at("REST");
    }
    
	// Left hand: 3 bits (2 down to 0)
	if (leftConnected) {
	    for (int i = 2; i >= 0; i--) {
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
    
    //printf("input string: %s\n", input.c_str());
        
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
        // Advance the phase
        float inverseSampleRate = 1.0f / context->audioSampleRate;
        *phase += 2.0f * M_PI * frequency * inverseSampleRate;
        if(*phase > 2.0f * M_PI) {
            *phase -= 2.0f * M_PI;
        }
    
        // Saxophone Timbre via Additive Synthesis
        float h1 = sin(*phase * 1.0f) * 1.00f; // Fundamental
        float h2 = sin(*phase * 2.0f) * 0.75f; // Octave (strong in conical bores)
        float h3 = sin(*phase * 3.0f) * 0.50f; // 12th
        float h4 = sin(*phase * 4.0f) * 0.40f; // 2nd Octave
        float h5 = sin(*phase * 5.0f) * 0.25f; // Major 3rd
        float h6 = sin(*phase * 6.0f) * 0.15f; // Fifth
    
        float rawSaxWave = h1 + h2 + h3 + h4 + h5 + h6;
    
        // mild asymmetrical waveshape to simulate the single reed
        float reedyWave = rawSaxWave + (rawSaxWave * rawSaxWave * 0.15f);
    
        // Smooth saturation 
        float saturated = tanh(reedyWave * 1.5f) / tanh(1.5f);
    
        // Scale down amplitude
        sample = saturated * 0.2f; 
    }
    
    return sample;
}

float generateSampleTrumpet(BelaContext *context, float *phase, float volumeControl, float frequency) {
    float sample = 0.0f;
    
    if(frequency > 0.0f) {
        // 1. Advance the phase
        float inverseSampleRate = 1.0f / context->audioSampleRate;
        *phase += 2.0f * M_PI * frequency * inverseSampleRate;
        if(*phase > 2.0f * M_PI) {
            *phase -= 2.0f * M_PI;
        }
    
        // Trumpet Timbre via Additive Synthesis (Harmonics 1 through 6)
        float h1 = sin(*phase * 1.0f) * 1.00f; // Fundamental
        float h2 = sin(*phase * 2.0f) * 0.60f; // Octave
        float h3 = sin(*phase * 3.0f) * 0.80f; // Perfect 5th above octave
        float h4 = sin(*phase * 4.0f) * 0.40f; // 2nd Octave
        float h5 = sin(*phase * 5.0f) * 0.50f; // Major 3rd above 2nd octave
        float h6 = sin(*phase * 6.0f) * 0.25f; // Perfect 5th above 2nd octave
    
        float rawBrassWave = h1 + h2 + h3 + h4 + h5 + h6;
    
        // non-linear saturation (tanh)
        float drive = 2.2f; 
        float saturated = tanh(rawBrassWave * drive) / tanh(drive);
    
        // Scale down amplitude to prevent digital clipping
        sample = saturated * 0.2f; 
    }
    
    return sample;
}

//=====================SETUP=====================
bool setup(BelaContext *context, void *userData)
{
    gSampleRate = context->audioSampleRate;   // needed by the guide tones

    gSerialTask = Bela_createAuxiliaryTask(serialTask, 50, "glove-serial");
    if (gSerialTask == 0) {
        printf("Error: could not create serial task\n");
        return false;
    }
    Bela_scheduleAuxiliaryTask(gSerialTask);
    
    mapping = tenorSaxMapping;
    rt_printf("Kinetone Initialized!\n");
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
        
    targetFrequency = fingersToFrequency(notes, mapping, leftConnected, leftFingers, rightConnected, rightFingers);//mapping.count("0100000") ? notes.at(mapping.at("0100000")) : notes.at("REST");//
    
    for(unsigned int n = 0; n < context->audioFrames; n++) {
        // Smooth controls per sample or per block cleanly
        secondsElapsed = (float)context->audioFramesElapsed / context->audioSampleRate;
        currentFrequency += (targetFrequency - currentFrequency) * 0.01f;
        rightRollPosition += (rightRollTarget - rightRollPosition) * kBendSmoothing;
        leftRollPosition = leftRoll;

        float bendFactor = rightPositionToBend(rightRollPosition, kPitchBendSemitones);
        float octaveFactor = leftPositionToOctave(leftRollPosition);
        float mutatedFrequency = currentFrequency * octaveFactor * bendFactor;
        
        float sample = generateSampleSin(context, &phase, volumeControl, mutatedFrequency);

        // mute the instrument during setup and mix in the guide tones
        gInstrumentGain += (instrumentTarget - gInstrumentGain) * 0.001f;
        sample = sample * gInstrumentGain + guideSample();
        
        audioWrite(context, n, 0, sample);
        audioWrite(context, n, 1, sample);
    }
}

void cleanup(BelaContext *context, void *userData) {}