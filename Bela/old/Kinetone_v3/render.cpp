// ==========================================================
// GESTURAL MIDI GLOVES — BELA RECEIVER
//
// Reads lines from the receiver ESP32 over USB serial:
//   NAME,fingers,roll,motion
//   fingers: index,middle,ring,pinky (1 = touching)
//   roll:    -180..180 (0 = palm-down zone, + = clockwise)
//   motion:  0..100 (not used yet)
// Splits and validates them in a background task, then maps
// finger patterns to a note and plays it.
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

// ==========================================
// SHARED HAND STATE (written by serial task, read by render)
// ==========================================

enum { kLeft = 0, kRight = 1 };


struct HandState {	
	std::atomic<bool>    connected{false};
	std::atomic<uint8_t> fingers{0};   // bit3 index, bit2 middle, bit1 ring, bit0 pinky
	std::atomic<int>     roll{0};      // -180..180, 0 = palm-down zone, + = clockwise
	std::atomic<int>     motion{0};    // 0..100
};

HandState gHands[2];

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

Clock::time_point gLastSeen[2];

void processLine(const std::string &line, Clock::time_point now)
{
    // Skip separators, diagnostics, blank lines
    if (line.empty() || line[0] == '#' || line == "---")
        return;

    std::vector<std::string> f = split(line, ',');
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

        // A hand whose lines have stopped arriving counts as disconnected
        for (int hand = 0; hand < 2; hand++) {
            if (gHands[hand].connected &&
                std::chrono::duration<double>(now - gLastSeen[hand]).count() > kStaleSeconds)
                    gHands[hand].connected = false;
        }
    }

    if (fd >= 0)
        close(fd);
}

AuxiliaryTask gSerialTask;

// ==========================================
// AUDIO
// ==========================================

// note mappings
const std::map<std::string, float> notes = {
    {"REST", 0.00f},
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
	{"1100000", "G4"},
	{"1111000", "F#4"},
	{"1110000", "F4"},
	{"1110010", "E4"},
	{"1110100", "D#4"},
	{"1110110", "D4"},
	{"1110111", "C4"}
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
        return notes.at("REST");   // CHANGED: previously built a 14-character string that never matched
    }

    // 1. Extract 4 bits from the left hand (bit 3 down to bit 0)
    if (leftConnected) {
        for (int i = 3; i >= 0; i--) {
            input += ((leftFingers >> i) & 1) ? '1' : '0';
        }
    } else {
        input += "0000";
    }

    // 2. Extract 3 bits from the right hand (bits 3, 2, and 1) to complete 7 characters
    if (rightConnected) {
        for (int i = 3; i >= 1; i--) {
            input += ((rightFingers >> i) & 1) ? '1' : '0';
        }
    } else {
        input += "000";
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
	    // 1. Advance the phase
	    float inverseSampleRate = 1.0f / context->audioSampleRate;
	    *phase += 2.0f * M_PI * frequency * inverseSampleRate;
	    if(*phase > 2.0f * M_PI) {
	        *phase -= 2.0f * M_PI;
	    }
	
	    // 2. Saxophone Timbre via Additive Synthesis (Full harmonic spectrum, rich mid-range)
	    float h1 = sin(*phase * 1.0f) * 1.00f; // Fundamental
	    float h2 = sin(*phase * 2.0f) * 0.75f; // Octave (strong in conical bores)
	    float h3 = sin(*phase * 3.0f) * 0.50f; // 12th
	    float h4 = sin(*phase * 4.0f) * 0.40f; // 2nd Octave
	    float h5 = sin(*phase * 5.0f) * 0.25f; // Major 3rd
	    float h6 = sin(*phase * 6.0f) * 0.15f; // Fifth
	
	    float rawSaxWave = h1 + h2 + h3 + h4 + h5 + h6;
	
	    // 3. Apply a mild asymmetrical waveshape to simulate the single reed
	    // Adding a slight squared term creates the characteristic reedy "bite"
	    float reedyWave = rawSaxWave + (rawSaxWave * rawSaxWave * 0.15f);
	
	    // Smooth saturation to keep it warm and prevent digital clipping
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
	
	    // 2. Trumpet Timbre via Additive Synthesis (Harmonics 1 through 6)
	    // Brass instruments feature strong upper harmonics giving them their bright "blat"
	    float h1 = sin(*phase * 1.0f) * 1.00f; // Fundamental
	    float h2 = sin(*phase * 2.0f) * 0.60f; // Octave
	    float h3 = sin(*phase * 3.0f) * 0.80f; // Perfect 5th above octave
	    float h4 = sin(*phase * 4.0f) * 0.40f; // 2nd Octave
	    float h5 = sin(*phase * 5.0f) * 0.50f; // Major 3rd above 2nd octave
	    float h6 = sin(*phase * 6.0f) * 0.25f; // Perfect 5th above 2nd octave
	
	    float rawBrassWave = h1 + h2 + h3 + h4 + h5 + h6;
	
	    // 3. Apply brass "bite" via non-linear saturation (tanh)
	    // This compresses the peaks and adds the characteristic metallic edge of a horn
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
        gSerialTask = Bela_createAuxiliaryTask(serialTask, 50, "glove-serial");
        if (gSerialTask == 0) {
                printf("Error: could not create serial task\n");
                return false;
        }
        Bela_scheduleAuxiliaryTask(gSerialTask);
        
		mapping = testMapping;
	    rt_printf("Kinetone Initialized!\n");
        return true;
}

//=====================RENDER=====================
void render(BelaContext *context, void *userData) {
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
        
        audioWrite(context, n, 0, sample);
        audioWrite(context, n, 1, sample);
    }
}

void cleanup(BelaContext *context, void *userData) {}