#include <Bela.h>
#include <cmath>
#include <map>
#include <iostream>



//=====================MAPS=====================
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

//=====================VARS=====================
// Phase accumulator
float phase = 0.0f;
float currentFrequency = 0.0f;
float targetFrequency = 0.0f;
float volumeControl = 0.8f; 

//setup variables
std::map<std::string, std::string> mapping;
const float kPitchBending = 5.5f;
std::string input = "0000000";
float leftRollPosition = 0; // [0, 2pi)
float rightRollPosition = 0; // [0, 2pi)
float secondsElapsed = 0;

//=====================FUNCTIONS=====================
float normalize(float position);
float leftPositionToOctave(float position);
float rightPositionToBend(float position, float k);
float generateSampleSin(BelaContext *context, float *phase, float volumeControl, float frequency, float leftRollPosition, float rightRollPosition);

//=====================SETUP=====================
bool setup(BelaContext *context, void *userData) {
	mapping = tenorSaxMapping;
    rt_printf("Kinetone Initialized!\n");
    return true;
}

//=====================RENDER=====================
void render(BelaContext *context, void *userData) {
    for(unsigned int n = 0; n < context->audioFrames; n++) {
        //total elapsed time in seconds
        secondsElapsed = (float)context->audioFramesElapsed / context->audioSampleRate;
        
    	//input variables
        input = "1000000";
        leftRollPosition = 0;
        rightRollPosition = 0;//sin(secondsElapsed*50);
        targetFrequency = mapping.count(input) ? notes.at(mapping.at(input)) : notes.at("REST");

        // Smooth frequency transitions
        currentFrequency += (targetFrequency - currentFrequency) * 0.05f;

        // Generate audio sample (Sine wave oscillator)
        float sample = generateSampleSin(context, &phase, volumeControl, currentFrequency, leftRollPosition, rightRollPosition);
        
        // Output the sound to stereo channels
        audioWrite(context, n, 0, sample);
        audioWrite(context, n, 1, sample);
    }
}

//=====================CLEANUP=====================
void cleanup(BelaContext *context, void *userData) {
    
}

//=====================FUNCTIONS=====================
float normalize(float position) {
	float normalized = position;
	
	while(normalized>2*M_PI)
		normalized-=2*M_PI;
	while(normalized<0)
		normalized+=2*M_PI;
	return normalized;
}

float leftPositionToOctave(float position) {
	return 1;
	/*
	float normalized = normalize(position);
	
	if(normalized > M_PI/4 && normalized <= 3*M_PI/4)
		return 0.5;
	if(normalized >= 5*M_PI/4 && normalized < 7*M_PI/4)
		return 2;
	return 1;*/
}

float rightPositionToBend(float position, float k) {
	float normalized = normalize(position);
	return (normalized > M_PI ? normalized - 2*M_PI : normalized) * k;
}

float generateSampleSin(BelaContext *context, float *phase, float volumeControl, float frequency, float leftRollPosition, float rightRollPosition) {
	float sample = 0.0f;
    if(currentFrequency > 0.0f) {
        float inverseSampleRate = 1.0f / context->audioSampleRate;
        *phase += 2.0f * M_PI * (frequency * leftPositionToOctave(leftRollPosition)
        	+ rightPositionToBend(rightRollPosition, kPitchBending)) * inverseSampleRate;
        if(*phase > 2.0f * M_PI) {
            *phase -= 2.0f * M_PI;
        }
        sample = sin(*phase) * volumeControl * 0.3f; // 0.3 scales amplitude
    }
    
    return sample;
}