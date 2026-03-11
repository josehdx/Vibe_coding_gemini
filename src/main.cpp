#include <Arduino.h>
#include <Control_Surface.h>


// Interfaces
USBMIDI_Interface usbmidi;
BluetoothMIDI_Interface blemidi;

// Hardware Constants
const pin_t pbPin = 13; // ADC A13 

// Manufacturer & User Data
// Deadzone: 1900 to 1980 (12-bit raw)
const int RAW_DZ_MIN = 1920;
const int RAW_DZ_MAX = 1940;
const int RAW_CENTER = (RAW_DZ_MIN + RAW_DZ_MAX) / 2; // 1940

// 12-bit ADC range is 0-4095. 
// Pitch Bend MIDI range is 0 to 16383 (14-bit), with 8192 as center.
const int MIDI_MIN = 0;
const int MIDI_CENTER = 8192;
const int MIDI_MAX = 16383;

int dynamicOffset = 0;

void setup() {
    Control_Surface.begin();

    analogReadResolution(12);
    
    // --- SELF-CENTERING CHECKUP ---
    // Calculate the offset based on the expected hardware center (1940)
    long runningSum = 0;
    for(int i = 0; i < 64; i++) {
        runningSum += analogRead(pbPin);
        delay(2);
    }
    int startupAverage = runningSum / 64;
    
    // If the stick is resting at 1950 but RAW_CENTER is 1940, offset is +10
    dynamicOffset = startupAverage - RAW_CENTER;
}

void loop() {
    Control_Surface.loop();

    static Timer<millis> timer = 20; // 50Hz update rate for smoothness
    if (timer) {
        int rawValue = analogRead(pbPin) - dynamicOffset;
        int pitchBendOutput = MIDI_CENTER;

        // Apply Logic based on TOCOS Deadzone
        if (rawValue < RAW_DZ_MIN) {
            // Map from absolute 0 to the start of the deadzone
            // Resulting in 0 to 8192
            pitchBendOutput = map(rawValue, 0, RAW_DZ_MIN, MIDI_MIN, MIDI_CENTER);
        } 
        else if (rawValue > RAW_DZ_MAX) {
            // Map from end of deadzone to absolute max 4095
            // Resulting in 8192 to 16383
            pitchBendOutput = map(rawValue, RAW_DZ_MAX, 4095, MIDI_CENTER, MIDI_MAX);
        }
        else {
            // Inside deadzone
            pitchBendOutput = MIDI_CENTER;
        }

        // Constrain to 14-bit limits
        pitchBendOutput = constrain(pitchBendOutput, 0, 16383);

        // Map MIDI 0-16383 to the library's internal Pitch Bend requirements
        // The Control Surface library uses -8192 to 8191 for its Pitch Bend function
        int finalValue = pitchBendOutput - 8192; 
        
        static int lastSent = -9999;
        if (finalValue != lastSent) {
            Control_Surface.sendPitchBend(CHANNEL_1, finalValue);
            lastSent = finalValue;
        }
    }
}