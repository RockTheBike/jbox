#include "JBox.h"  // must be present
#include <math.h>
#define VERSION "Demo of multiple free from Adafruit_NeoPixel copy constructor"
#include <Adafruit_NeoPixel.h>

#define NUM_AMP_SENSORS 5
#define IND_INTERVAL 500
unsigned long time = 0;
unsigned long lastIndicatorTime = 0;
Adafruit_NeoPixel strips[NUM_AMP_SENSORS] = {
		Adafruit_NeoPixel(12, 3, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(12, 4, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(12, 5, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(12, 6, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(12, 7, NEO_GRB + NEO_KHZ800)
};

void setup() {
	Serial.begin(57600);
	Serial.println(VERSION);
	for( int i=5; i; i-- ) {
		delay(1000);
		Serial.print(i);
		Serial.print(' ');
	}
	Serial.println("\nHere we go...");
}

// passing Adafruit_NeoPixel by reference would avoid heap corruption
uint32_t Wheel(Adafruit_NeoPixel strip, byte WheelPos) {
	if (WheelPos < 85) {
		return strip.Color(255 - WheelPos * 3, WheelPos * 3, 0);
	} else if (WheelPos < 170) {
		WheelPos -= 85;
		return strip.Color(0, 255 - WheelPos * 3, WheelPos * 3);
	} else {
		WheelPos -= 170;
		return strip.Color(WheelPos * 3, 0, 255 - WheelPos * 3);
	}
}


void doIndRamp(uint8_t s){
    static unsigned char hue = 0;
    Serial.print("About to calculate color for hue ");
    Serial.println(hue);
    uint32_t color = Wheel(strips[s], hue);
    Serial.print("Calculated color 0x");
    Serial.println(color,HEX);
    hue++;
}

void doIndicators(){
		for(int i = 0; i < NUM_AMP_SENSORS; i++){
			doIndRamp(i);
		}
}

void loop() {
	time = millis();
	if(time - lastIndicatorTime > IND_INTERVAL){
		lastIndicatorTime = time;
		doIndicators();
	}
}
