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
float generateSampleSax(BelaContext *context, float *phase, float volumeControl, float frequency, float leftRollPosition, float rightRollPosition);
float generateSampleTrumpet(BelaContext *context, float *phase, float volumeControl, float frequency, float leftRollPosition, float rightRollPosition);

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
        float sample = generateSampleSax(context, &phase, volumeControl, currentFrequency, leftRollPosition, rightRollPosition);
        
        /*
        // Inside your render loop:
		float sample = 0.0f;
		
		if(currentFrequency > 0.0f) {
		    // 1. Advance the phase
		    float inverseSampleRate = 1.0f / context->audioSampleRate;
		    phase += 2.0f * M_PI * currentFrequency * inverseSampleRate;
		    if(phase > 2.0f * M_PI) {
		        phase -= 2.0f * M_PI;
		    }
		
		    // 2. Flute Timbre: Dominant fundamental with very soft upper harmonics
		    // Flutes lack the strong mid/high harmonics of brass or reeds
		    float h1 = sin(phase * 1.0f) * 1.00f; // Fundamental (dominant)
		    float h2 = sin(phase * 2.0f) * 0.12f; // Soft octave
		    float h3 = sin(phase * 3.0f) * 0.04f; // Faint 12th
		
		    float rawFluteWave = h1 + h2 + h3;
		
		    // 3. Add a touch of white noise to simulate breath turbulence across the embouchure hole
		    float breathNoise = ((float)rand() / RAND_MAX * 2.0f - 1.0f) * 0.04f;
		    
		    // Combine tone and breath
		    float finalWave = rawFluteWave + breathNoise;
		
		    // Scale down amplitude to keep it smooth and prevent clipping
		    sample = finalWave * 0.25f; 
		}
		*/
        
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
	float mutatedFrequency = (frequency * leftPositionToOctave(leftRollPosition)
        	+ rightPositionToBend(rightRollPosition, kPitchBending));
    
    if(currentFrequency > 0.0f) {
        float inverseSampleRate = 1.0f / context->audioSampleRate;
        *phase += 2.0f * M_PI * mutatedFrequency * inverseSampleRate;
        if(*phase > 2.0f * M_PI) {
            *phase -= 2.0f * M_PI;
        }
        sample = sin(*phase) * volumeControl * 0.3f; // 0.3 scales amplitude
    }
    
    return sample;
}

float generateSampleSax(BelaContext *context, float *phase, float volumeControl, float frequency, float leftRollPosition, float rightRollPosition) {
	float sample = 0.0f;
	float mutatedFrequency = (frequency * leftPositionToOctave(leftRollPosition)
    	+ rightPositionToBend(rightRollPosition, kPitchBending));
		
	if(mutatedFrequency > 0.0f) {
	    // 1. Advance the phase
	    float inverseSampleRate = 1.0f / context->audioSampleRate;
	    *phase += 2.0f * M_PI * mutatedFrequency * inverseSampleRate;
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

float generateSampleTrumpet(BelaContext *context, float *phase, float volumeControl, float frequency, float leftRollPosition, float rightRollPosition) {
	float sample = 0.0f;
	float mutatedFrequency = (frequency * leftPositionToOctave(leftRollPosition)
    	+ rightPositionToBend(rightRollPosition, kPitchBending));
	
	
	if(mutatedFrequency > 0.0f) {
	    // 1. Advance the phase
	    float inverseSampleRate = 1.0f / context->audioSampleRate;
	    *phase += 2.0f * M_PI * mutatedFrequency * inverseSampleRate;
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