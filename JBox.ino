#include "JBox.h"  // must be present
#include <math.h>

/// *********** defines

#define VERSION "Rock the Bike Junction Box 1.0"

#define ENABLE_PROTECT 0
#define ENABLE_INDICATORS 1
#define ENABLE_SENSE 1
#define DEBUG 1

#if ENABLE_INDICATORS
#include <Adafruit_NeoPixel.h>
#endif

#define AVG_CYCLES 200.0
#define BLINK_INTERVAL 1000
#define DISPLAY_INTERVAL 1000
#define ENERGY_INTERVAL 0
#define NUM_AMP_SENSORS 5
#define PIN_LED 13

#define VOLTCOEFF 13.179 // correct value for new blue arbduino v2
#define AMPCOEFF 8.0682 // 583 - 512 = 71; 71 / 8.8 amps = 8.0682
#define AMPOFFSET 512.0 // when current sensor is at 0 amps this is the ADC value

#if ENABLE_SENSE
#define PIN_VOLTS A0

#define PIN_AMPS { A4, A1, A3, A2, A5 }
#define SENSOR_TYPES { 50, 50, 50, 50, 50 }
//#define OFFSETS { -24, -24, -24, -24, -24 }
#endif

#if ENABLE_PROTECT
#define PIN_PROTECT 2
#define PROTECT_INTERVAL 0
#define PROTECT_CUTOFF 53.0
#define PROTECT_RECOVER 50.0
#endif

#if ENABLE_INDICATORS
#define PIN_PIXELS {3, 4, 5, 6, 7}
#define NUM_PIXELS 14 // number LEDs per column (2 columns: 1 for power, 1 for energy)
#define IND_INTERVAL 500
#define IND_BLINK_INTERVAL 300
#define IND_VOLT_LOW -1
#define IND_VOLT_HIGH 50.0

#define STATE_OFF 0
#define STATE_ON 1
#define STATE_BLINK_LOW 2
#define STATE_BLINK_HIGH 3
#define STATE_RAMP 4
#define STATE_RESET 5

#endif

//// *********** globals

#if ENABLE_SENSE
int pinAmps[] = PIN_AMPS;
int sensorType[] = SENSOR_TYPES;
//int sensorOffset[NUM_AMP_SENSORS] = OFFSETS;

unsigned int voltAdc = 0;
float voltAdcAvg = 0;
float volts = 0.0;

int ampsRaw[NUM_AMP_SENSORS] = { 0 };
float amps[NUM_AMP_SENSORS] = { 0.0 };
float watts[NUM_AMP_SENSORS] = { 0.0 };
float energy[NUM_AMP_SENSORS] = { 0.0 };
float totalWatts = 0.0;
float totalEnergy = 0.0;
#endif

//typedef unsigned long ulong;
unsigned long time = 0;
unsigned long lastTime = 0;
unsigned long lastVolt = 0;
unsigned long lastDisplay = 0;
unsigned long lastBlink = 0;
unsigned long lastEnergy = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastFastBlinkTime = 0;

boolean isBlinking = false;  // LED13
boolean blinkState = false;  // indicators
boolean fastBlinkState = false;  // indicators
unsigned long sampleCount = 0;
boolean enableRawMode = false;
boolean enableAutoDisplay = false;

#if ENABLE_PROTECT
boolean isProtected = false;
unsigned long lastProtect = 0;
#endif

#if ENABLE_INDICATORS
int pinLEDs[] = PIN_PIXELS;
unsigned long lastIndicatorTime = 0;
unsigned long lastIndicatorBlinkTime = 0;
int indState = STATE_RAMP;
int indStates[NUM_AMP_SENSORS];
boolean indBlinkState = false;
uint32_t indColor[NUM_AMP_SENSORS] = {0};

// based directly from strandtest.ino, the example file supplied with Adafruit Neopixel libraries.
// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
// NEO_RGB Pixels are wired for RGB bitstream
// NEO_GRB Pixels are wired for GRB bitstream
// NEO_KHZ400 400 KHz bitstream (e.g. FLORA pixels)
// NEO_KHZ800 800 KHz bitstream (e.g. High Density LED strip)

Adafruit_NeoPixel strips[NUM_AMP_SENSORS] = {
		Adafruit_NeoPixel(12, 3, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(12, 4, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(12, 5, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(12, 6, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(12, 7, NEO_GRB + NEO_KHZ800)
};


#endif


////// *********** functions  -  each function is implemented before called, and loop() is at the end, to facilitate using alternative C compilers & IDEs.

//The setup function is called once at startup of the sketch
void setup() {

	Serial.begin(57600);

#if ENABLE_SENSE
	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_VOLTS, INPUT); // voltage ADC
	for (int i = 0; i < NUM_AMP_SENSORS; i++) {
		pinMode(pinAmps[i], INPUT);
	}
#endif

#if ENABLE_PROTECT
	pinMode(PIN_PROTECT, OUTPUT);
#endif

#if ENABLE_INDICATORS
	for (int i = 0; i < NUM_AMP_SENSORS; i++) {
		strips[i].begin();
		//setStrip(strips[i], 0,0,0);
		strips[i].show(); // Initialize all pixels to 'off'
	}
#endif

}

#if ENABLE_INDICATORS

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
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

// Fill the dots one after the other with a color
void colorWipe(Adafruit_NeoPixel strip, uint32_t c, uint8_t wait) {
	for (uint16_t i = 0; i < strip.numPixels(); i++) {
		strip.setPixelColor(i, c);
		//delay(wait);
	}
	strip.show();
}
void rainbow(Adafruit_NeoPixel strip, uint8_t wait) {
	uint16_t i, j;
	for (j = 0; j < 256; j++) {
		for (i = 0; i < strip.numPixels(); i++) {
			strip.setPixelColor(i, Wheel(strip, (i + j) & 255));
		}
		strip.show();
		delay(wait);
	}
}
// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(Adafruit_NeoPixel strip, uint8_t wait) {
	uint16_t i, j;
	for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
		for (i = 0; i < strip.numPixels(); i++) {
			strip.setPixelColor(i,
					Wheel(strip, ((i * 256 / strip.numPixels()) + j) & 255));
		}
		strip.show();
		delay(wait);
	}
}

void setStrip(Adafruit_NeoPixel strip, uint8_t r, uint8_t g, uint8_t b){
	for (uint16_t i = 0; i < strip.numPixels(); i++) {
		strip.setPixelColor(i, r, g, b);
	}
	strip.show();
}


void setPixel(Adafruit_NeoPixel strip, uint16_t pixel, uint32_t c){
	strip.setPixelColor(pixel, c);
	//strip.show();
}

void setAll(int r, int g, int b) {
	setStrip(strips[0], r, g, b);
	setStrip(strips[1], r, g, b);
	setStrip(strips[2], r, g, b);
	setStrip(strips[3], r, g, b);
	setStrip(strips[4], r, g, b);
}

void doIndBlink(){

	if(time - lastIndicatorBlinkTime > IND_BLINK_INTERVAL){
		lastIndicatorBlinkTime = time;
		indBlinkState = !indBlinkState;

//		Serial.print("ind blink ");
//		//Serial.print(indState);
//		//Serial.print(" ");
//		Serial.println(blinkCount++);

		uint16_t j = NUM_PIXELS;
		int i = NUM_AMP_SENSORS;

		// turn all pixels off:
		if(!indBlinkState){
			setAll( 0,0,0);
		}
		else {
			if(indState == STATE_BLINK_LOW){
				setAll(255,0,0);
			}else if(indState == STATE_BLINK_HIGH){
				setAll(255,255,255);
			}
		}

		// no need to show strip as doIndicators will handle it
		//strip.show();
	}

}

void doIndRamp(uint8_t s){

    float temp = 0.0;

    if(watts[s] > 3){ // TODO change to a gentler slope than a natural log
    	temp = log(watts[s]);
    	// TODO maybe change algorithm for kids here
//    	if(watts[s] < 30)
//    		temp = temp / 2;
    }

    unsigned char hue = (temp / 7.0) * 170.0;
    if(hue < 1) hue = 1;

    uint32_t color = Wheel(strips[s], hue);

    long itemp = (long)temp; // turn log(watts) into the LED index
    long remainder = (long)((temp - (long)temp) * 255.0); // get the remainder to light the next LED less

#if DEBUG
    if (!enableAutoDisplay) { // turn off when enabling autoDisplay
      if (s == 0) Serial.println(""); // newline once per report of all five ramps
      Serial.print(" ra");
      Serial.print(s);
      Serial.print(" in");
      Serial.print(indState);
      Serial.print(" te");
      Serial.print(temp);
      Serial.print(" hu");
      Serial.print(hue);
      Serial.print(" co");
      Serial.print(color,HEX);
      Serial.print(" it");
      Serial.print(itemp);
      Serial.print(" re");
      Serial.print(remainder);
    }
#endif


    for(int i = 0; i < NUM_PIXELS; i++){
    	if(i <= itemp){
            strips[s].setPixelColor(i, color);
    	} else {
    		strips[s].setPixelColor(i, 0,0,0); // set others dark
    	}
    }

    strips[s].show();
}

void showStrips(){

	for(int i = 0; i < NUM_AMP_SENSORS; i++){
		strips[i].show();
	}
}

void doIndicators(){

#if ENABLE_SENSE
	// test for state change
	if(volts < IND_VOLT_LOW){
		indState = STATE_BLINK_LOW;
	} else if(volts < IND_VOLT_HIGH){
		indState = STATE_RAMP;
	} else if (volts >= IND_VOLT_HIGH){
		indState = STATE_BLINK_HIGH;
	}
#endif

	if(indState == STATE_BLINK_HIGH){
		doIndBlink();
	} else {
		for(int i = 0; i < NUM_AMP_SENSORS; i++){
			doIndRamp(i);
		}
	}

}

#endif // END ENABLE_INDICATORS

#if ENABLE_PROTECT

void setProtect(boolean protect){
	if(protect){
		digitalWrite(PIN_PROTECT, HIGH);
		isProtected = true;
	} else {
		digitalWrite(PIN_PROTECT, LOW);
		isProtected = false;
	}
}

void doProtect(){
	if(volts > PROTECT_CUTOFF)
	{
		setProtect(true);
	}
	else if(isProtected && volts < PROTECT_RECOVER)
	{
		setProtect(false);
	}
}

#endif // END ENABLE_PROTECT

#if ENABLE_SENSE
float averageF(float val, float avg){
	if(avg == 0)
		avg = val;
	return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
}

// faster than floats; values can be multiplied by 10 to get 0.1 accuracy
long averageI(int val, int avg){
	if(avg == 0){
		avg = val;
	}
	return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
}

int volts2adc(float v){
	return (int)(v * VOLTCOEFF);
}

float adc2volts(float adc){
	return adc / VOLTCOEFF;
}

float adc2pinvolts(float adc){
	// v = adc * 5 / 1024 = adc * 0.0048828125
	return adc * 0.0048828125;
}

float adc2amps(int adc, int sensorType){
	return ((float)adc - AMPOFFSET) * (sensorType == 100 ? 0.244379276637341 : 0.1220703125);
}

float amps2adc(float amps, int sensorType){
	// adc/A = 8.192 adc/A
	return AMPOFFSET + amps * (sensorType == 100 ? 4.092 : 8.192);
}


void doEnergy(){
	sampleCount++;

	float temp = 0.0;
	int tempI = 0;
	int i = 0;
	int j = 0;
	totalWatts = 0;

	//measure volts
	for(int i = 0; i < 3; i++){
		voltAdc = analogRead(PIN_VOLTS);
		temp = adc2volts((float)voltAdc);
		volts = averageF(temp, volts);
	}

	float timeDiff = time - lastEnergy;
	float timeDiffSecs = timeDiff / 1000.0;

#if DEBUG
	//volts = 30.0;
#endif

	// measure amps and calc energy
	for(i = 0; i < NUM_AMP_SENSORS; i++){

		for(int i = 0; i < 3; i++){
			ampsRaw[i] = analogRead(pinAmps[i]);
			//tempI = ampsRaw[i] + sensorOffset[i];
			tempI = ampsRaw[i];
			if(tempI > 505 && tempI < 516)
				tempI = 511;
			temp = - adc2amps(tempI, sensorType[i]);
			amps[i] = averageF(temp, amps[i]);
		}

		//calc watts and energy
		watts[i] = volts * amps[i];
		totalWatts += watts[i];
		float wattsecs = watts[i] * timeDiffSecs;
		float watthrs = wattsecs / 3600;
		energy[i] += wattsecs; // watt secs
		totalEnergy += watthrs;  // watt hours

	}
	lastEnergy = time;

#if DEBUG
	//watts[0] = 50.0;
	//watts[1] = 100.0;
#endif

}


void resetEnergy(int input){
	// if input == -1, reset all
	if(input == -1){
		for(int i = NUM_AMP_SENSORS; i > 0; --i){
			energy[i] = 0;
		}
		//totalEnergy = 0;
	} else { // otherwise, just reset the one input
		energy[input] = 0;
	}
}
#endif


void doDisplay(){
#if ENABLE_SENSE
	Serial.print("{VOLTS: ");
	Serial.print(volts, 2);
	Serial.print(", RAW: ");
	Serial.print(voltAdc);

	for(int i = 0; i < NUM_AMP_SENSORS; i++){
		Serial.print(", ");
		Serial.print(i);
		Serial.print(": ");
		if(enableRawMode)
			Serial.print(ampsRaw[i]);
		else
			Serial.print(watts[i], 0);
	}
	Serial.print(", TOTAL_WATTS: ");
	Serial.print(totalWatts, 2);
	Serial.print(", TOTAL_WATTHRS: ");
	Serial.print(totalEnergy, 2);
#endif
#if ENABLE_PROTECT
	if(isProtected)
		Serial.print(", PROTECT: ON");
#endif
	Serial.println("}");
	//resetEnergy();
}

void doSerial(int in){
	// get incoming byte:
	switch(in){
    case 'a':
      enableAutoDisplay = !enableAutoDisplay;
      break;
	case 'd':
		doDisplay();
		break;
	case 'r':
		enableRawMode = !enableRawMode;
		break;

#if ENABLE_PROTECT
	case 'p':
		setProtect(!isProtected);
		break;
#endif
#if ENABLE_SENSE
	case 'x':
		resetEnergy(-1);
		break;
#endif
	case 'z': // version
		Serial.println(VERSION);
		break;
	default:
		break;
	}
}

void doBlink() {
	//Serial.print("blink ");
	//Serial.println(blinkCount++);
	isBlinking = !isBlinking;
	digitalWrite(PIN_LED, isBlinking);
}

// The loop function is called in an endless loop
void loop() {

	time = millis();

	if (time - lastBlink > BLINK_INTERVAL) {
		lastBlink = time;
		doBlink();
	}

	if (enableAutoDisplay && time - lastDisplay > DISPLAY_INTERVAL) {
		lastDisplay = time;
		doDisplay();
	}

//	if (time - lastBlinkTime > 600){
//		blinkState = !blinkState;
//		lastBlinkTime=time;
//	}
//
//	if (time - lastFastBlinkTime > 120){
//		fastBlinkState = !fastBlinkState;
//		lastFastBlinkTime=time;
//	}

#if ENABLE_SENSE
	if(time - lastEnergy > ENERGY_INTERVAL){
		lastEnergy = time;
		doEnergy();
	}
#endif

#if ENABLE_PROTECT
	if(time - lastProtect > PROTECT_INTERVAL){
		lastProtect = time;
		doProtect();
	}
#endif

#if ENABLE_INDICATORS
	if(time - lastIndicatorTime > IND_INTERVAL){
		lastIndicatorTime = time;
		doIndicators();
	}
#endif

	if (Serial.available() > 0) {
		 int in = Serial.read();
		 doSerial(in);
	}

}
