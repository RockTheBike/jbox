#include "JBox.h"  // must be present
#include <math.h>
#include <EEPROM.h>

/// *********** defines

#define ENABLE_PROTECT 1
#define ENABLE_INDICATORS 1
#define ENABLE_SENSE 1
#define ENABLE_DIMMING 1
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
#define PIN_VOLTS2 A1 // stolen from PIN_AMPS[1]

#define PIN_AMPS { A4, A4, A3, A2, A5 }
#define OFFSETS { -4, -4, -5, -4, -5 }
#define NOISYZERO 0.2  // assume any smaller measurement should be 0
#endif

#if ENABLE_PROTECT
#define PIN_PROTECT 2
#define PROTECT_INTERVAL 0
#define PROTECT_CUTOFF 45.0
#define PROTECT_RECOVER 40.0
#endif

#if ENABLE_INDICATORS
#define PIN_PIXELS {3, 4, 5, 6, 7}
#define NUM_POWER_PIXELS 7  // number LEDs for power
#define NUM_ENERGY_PIXELS 7  // number LEDs for energy
#define NUM_PIXELS (NUM_POWER_PIXELS+NUM_ENERGY_PIXELS)  // number LEDs per bike
#define IND_INTERVAL 100
#define BUTTON_CHECK_INTERVAL 100
#define IND_BLINK_INTERVAL 300
#define IND_VOLT_LOW -1
#define IND_VOLT_HIGH 50.0

// scale the logarithmic displays:
// (we assume a relaxed pedaler produces 60W)
// barely turning the cranks produces 10W (not much more than noise)
#define MIN_POWER 10
// a sprinting athlete should just barely reach the top
#define MAX_POWER 1000

#ifndef WIMPIFY
// 15sec of relaxed pedaling should trigger minimally visible glow
#define MIN_ENERGY (float)(60*15)
// earn a smoothie with equivalent of a half-hour of relaxed pedaling
// ie 60W * 0.5hr * 3600s/hr
#define MAX_ENERGY (float)(60*0.5*3600)
#else
// re-tune for testers and other wimps:  earn a smoothie in seconds!
#define MIN_ENERGY (float)(60*5)
#define MAX_ENERGY (float)(60*10)
#endif

uint32_t ENERGY_COLORS[] = {
  Adafruit_NeoPixel::Color(0,0,0),
  Adafruit_NeoPixel::Color(255,0,0),
  Adafruit_NeoPixel::Color(0,255,0),
  Adafruit_NeoPixel::Color(255,0,255),
  Adafruit_NeoPixel::Color(255,128,0),
  Adafruit_NeoPixel::Color(0,255,255) };
#define NUM_ENERGY_COLORS (sizeof(ENERGY_COLORS)/sizeof(*ENERGY_COLORS))

#if ENABLE_DIMMING
#define BRIGHTNESS_EEPROM_ADDRESS 10
#endif


#define STATE_OFF 0
#define STATE_ON 1
#define STATE_BLINK_LOW 2
#define STATE_BLINK_HIGH 3
#define STATE_RAMP 4

#endif

//// *********** globals

#if ENABLE_SENSE
int pinAmps[] = PIN_AMPS;
int sensorOffset[NUM_AMP_SENSORS] = OFFSETS;

unsigned int voltAdc = 0;
float volts = 0.0;
float volts2 = 0.0;

int ampsRaw[NUM_AMP_SENSORS] = { 0 };
float amps[NUM_AMP_SENSORS] = { 0.0 };
float watts[NUM_AMP_SENSORS] = { 0.0 };
float energy[NUM_AMP_SENSORS] = { 0.0 };  // watt secs
unsigned long resetTime2,resetTime3;  // time since reset
float totalWatts = 0.0;
float totalEnergy = 0.0;  // watt hours
#endif

//typedef unsigned long ulong;
unsigned long time = 0;
unsigned long lastTime = 0;
unsigned long lastVolt = 0;
unsigned long lastDisplay = 0;
unsigned long lastBlink = 0;
unsigned long lastEnergy = 0;
unsigned long lastButtonCheckTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastFastBlinkTime = 0;

boolean isBlinking = false;  // LED13
boolean blinkState = false;  // indicators
boolean fastBlinkState = false;  // indicators
unsigned long sampleCount = 0;
boolean enableRawMode = false;
boolean enableAutoDisplay = true;

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
		Adafruit_NeoPixel(NUM_PIXELS, 3, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(NUM_PIXELS, 4, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(NUM_PIXELS, 5, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(NUM_PIXELS, 6, NEO_GRB + NEO_KHZ800),
		Adafruit_NeoPixel(NUM_PIXELS, 7, NEO_GRB + NEO_KHZ800)
};


#endif

#if ENABLE_DIMMING
float brightness = 1.0;
#endif


////// *********** functions  -  each function is implemented before called, and loop() is at the end, to facilitate using alternative C compilers & IDEs.

//The setup function is called once at startup of the sketch
void setup() {

	Serial.begin(57600);
        Serial.println("Rider 1 Wattage, Rider 1 Energy, Rider 1 Time,  Rider 2 Wattage, Rider 2 Energy, Rider 2 Time");

#if ENABLE_SENSE
	pinMode(PIN_LED, OUTPUT);
#endif

// To avoid a floating voltage that will burn out the transistor, we set the
// pin to output, even when ENABLE_PROTECT is false.
	pinMode(PIN_PROTECT, OUTPUT);

#if ENABLE_DIMMING
	brightness = EEPROM.read(BRIGHTNESS_EEPROM_ADDRESS) / 255.0;
        if(brightness<0.1) brightness=1;  // assume the EEPROM was corrupted
#endif

#if ENABLE_INDICATORS
	for (int i = 0; i < NUM_AMP_SENSORS; i++) {
		strips[i].begin();
		//setStrip(strips[i], 0,0,0);
		strips[i].show(); // Initialize all pixels to 'off'
	}
#endif

        time = millis();
        resetTime2 = time;
        resetTime3 = time;
        // this prevents boot! resetEnergy(-1); // reset energy (and resetTime) for all
}

#if ENABLE_INDICATORS

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(const Adafruit_NeoPixel& strip, byte WheelPos) {
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

void setStrip(Adafruit_NeoPixel& strip, uint8_t r, uint8_t g, uint8_t b){
	for (uint16_t i = 0; i < strip.numPixels(); i++) {
		strip.setPixelColor(i, dim(r), dim(g), dim(b));
	}
	strip.show();
}


void setAll(int r, int g, int b) {
	for( int i=0; i<NUM_AMP_SENSORS; i++ )
		setStrip(strips[i], r, g, b);
}

void doIndBlink(){

	if(time - lastIndicatorBlinkTime > IND_BLINK_INTERVAL){
		lastIndicatorBlinkTime = time;
		indBlinkState = !indBlinkState;

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
	float ledstolight;

	// the power LEDs
	ledstolight = logPowerRamp(watts[s]);
	if( ledstolight > NUM_POWER_PIXELS ) ledstolight=NUM_POWER_PIXELS;
	unsigned char hue = ledstolight/NUM_POWER_PIXELS * 170.0;
	uint32_t color = Wheel(strips[s], hue<1?1:hue);
	static const uint32_t dark = Adafruit_NeoPixel::Color(0,0,0);
	doFractionalRamp(s, 0, NUM_POWER_PIXELS, ledstolight, color, dark);

	// the energy LEDs
	int full_smoothies = energy[s]/MAX_ENERGY;
	float partial_smoothie = energy[s] - full_smoothies * MAX_ENERGY;
	ledstolight = logEnergyRamp(partial_smoothie);
	if( ledstolight > NUM_ENERGY_PIXELS ) ledstolight=NUM_ENERGY_PIXELS;
	uint32_t curcolor = ENERGY_COLORS[full_smoothies%NUM_ENERGY_COLORS];
	uint32_t nextcolor = ENERGY_COLORS[(full_smoothies+1)%NUM_ENERGY_COLORS];
	doBackwardsFractionalRamp(s, NUM_POWER_PIXELS, NUM_ENERGY_PIXELS,
	  ledstolight, nextcolor, curcolor );

	// and show 'em
	strips[s].show();
}

void doFractionalRamp(uint8_t s, uint8_t offset, uint8_t num_pixels, float ledstolight, uint32_t firstColor, uint32_t secondColor){
	for( int i=0,pixel=offset; i<=num_pixels; i++,pixel++ ){
		uint32_t color;
		if( i<(int)ledstolight )  // definitely firstColor
		    color = firstColor;
		else if( i>(int)ledstolight )  // definitely secondColor
		    color = secondColor;
		else  // mix the two proportionally
		    color = weighted_average_of_colors( firstColor, secondColor, ledstolight-(int)ledstolight);
		strips[s].setPixelColor(pixel, dim(color));
	}
}

// useful for the upside-down energy LEDs
void doBackwardsFractionalRamp(uint8_t s, uint8_t offset, uint8_t num_pixels, float ledstolight, uint32_t firstColor, uint32_t secondColor){
	doFractionalRamp(s, offset, num_pixels, num_pixels-ledstolight, secondColor, firstColor);
}

// Yay, a closed form solution, and it's even got meaningful parameters!

float logPowerRamp( float p ) {
	float l = log(p/MIN_POWER)*NUM_POWER_PIXELS/log(MAX_POWER/MIN_POWER);
	return l<0 ? 0 : l;
}

float logEnergyRamp( float e ) {
	float l = log(e/MIN_ENERGY)*NUM_ENERGY_PIXELS/log(MAX_ENERGY/MIN_ENERGY);
	return l<0 ? 0 : l;
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

void doButtonCheck() {
	// Need some time between pinMode and digitalRead for stuff to settle.
	// Luckily, working on the other pins seems to be enough.
	for( int s=0; s<NUM_AMP_SENSORS; s++ )
		pinMode( pinLEDs[s], INPUT_PULLUP );
	for( int s=0; s<NUM_AMP_SENSORS; s++ )
		// button closes data line to ground
		if( ! digitalRead(pinLEDs[s]) ) {
			resetEnergy( s );
			// Serial.print("resetEnergy for bike ");
			// Serial.println(s);
		}
	for( int s=0; s<NUM_AMP_SENSORS; s++ )
		strips[s].begin();
}

#if ENABLE_PROTECT

void setProtect(boolean protect){
	digitalWrite(PIN_PROTECT, protect ? HIGH : LOW);
	isProtected = protect;
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

float adc2volts(float adc){
	return adc / VOLTCOEFF;
}

float adc2pinvolts(float adc){
	// v = adc * 5 / 1024 = adc * 0.0048828125
	return adc * 0.0048828125;
}

float adc2amps(int adc, int sensorOffset=0){
	return - (adc - sensorOffset - AMPOFFSET) / AMPCOEFF;
}


void doEnergy(){
	sampleCount++;

	float temp = 0.0;
	int i = 0;
	int j = 0;
	totalWatts = 0;

	//measure volts
	for(j = 0; j < 3; j++){
		voltAdc = analogRead(PIN_VOLTS);
		temp = adc2volts((float)voltAdc);
		volts = averageF(temp, volts);
	}
	for(j = 0; j < 3; j++){
		voltAdc = analogRead(PIN_VOLTS2);
		temp = adc2volts((float)voltAdc);
		volts2 = averageF(temp, volts2);
	}

	float timeDiff = time - lastEnergy;
	float timeDiffSecs = timeDiff / 1000.0;

	// measure amps and calc energy
	for(i = 0; i < NUM_AMP_SENSORS; i++){

		for(j = 0; j < 3; j++){
			ampsRaw[i] = analogRead(pinAmps[i]);
			temp = adc2amps(ampsRaw[i], sensorOffset[i]);
			amps[i] = averageF(temp, amps[i]);
		}
		// we assume anything near or below zero is a reading error
		if( amps[i] < NOISYZERO ) amps[i] = 0;

		//calc watts and energy
		if (pinAmps[i]==A3) {
                  watts[i] = volts * amps[i]; // volts only applies to A3 input
                } else {
                  watts[i] = volts2 * amps[i]; // all others are on A1 voltage pin
                }
		totalWatts += watts[i];
		float wattsecs = watts[i] * timeDiffSecs;
		float watthrs = wattsecs / 3600;
		energy[i] += wattsecs; // watt secs
		totalEnergy += watthrs;  // watt hours

	}
	lastEnergy = time;

}

#if ENABLE_DIMMING
void readBrightness( int c ){
	if( c=='0' || c=='b' )
		brightness = 1.0;
	else if( '1'<=c && c<='9' )
		brightness = (c-'0')/10.0;
	// otherwise ignore
	Serial.print("Brightness is now ");
	Serial.print((int)(brightness*100));
	EEPROM.write( BRIGHTNESS_EEPROM_ADDRESS, brightness*255 );
	Serial.println("%");
}
#endif

void resetEnergy(int input){
	// if input == -1, reset all
	if(input == -1){
		for(int i=0; i<NUM_AMP_SENSORS; i--){
			energy[i] = 0;
                        resetTime2 = time;
                        resetTime3 = time;
		}
	} else { // otherwise, just reset the one input
		energy[input] = 0;
                if (input == 2) resetTime2 = time;
                if (input == 3) resetTime3 = time;
	}
}
#endif


// Rider 1 Wattage, Rider 1 Energy, Rider 1 Time,  Rider 2 Wattage, Rider 2 Energy, Rider 2 Time
// 30, 8.9, 240, 25, 20.0, 60
// 32, 9.0, 241, 23, 20.1, 61
// 30, 9.1, 242, 30, 20.1, 62
// ...
void doDisplay(){
	Serial.print(watts[3], 0);
	Serial.print(", ");
	Serial.print(energy[3], 1);
	Serial.print(", ");
	Serial.print((time - resetTime3) / 1000);
	Serial.print(", ");

	Serial.print(watts[2], 0);
	Serial.print(", ");
	Serial.print(energy[2], 1);
	Serial.print(", ");
	Serial.println((time - resetTime2) / 1000);

        return;
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

#if ENABLE_SENSE
	if(time - lastEnergy > ENERGY_INTERVAL){
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

	if(time - lastButtonCheckTime > BUTTON_CHECK_INTERVAL){
		lastButtonCheckTime = time;
		doButtonCheck();
	}
}

// TODO:  chop this and replace it with Adafruit_NeoPixel::setBrightness
// hacky utilities to dim the LEDs so they don't hurt our eyes
uint8_t dim(uint8_t c){  // a single color chanel
#if ENABLE_DIMMING
	return (uint8_t)(brightness*c);
#else
	return c;
#endif
}
uint32_t dim(uint32_t c){  // a full RGB color
#if ENABLE_DIMMING
	return Adafruit_NeoPixel::Color(
	dim( (uint8_t)( (c>>16) & 0xff ) ),
	dim( (uint8_t)( (c>>8 ) & 0xff ) ),
	dim( (uint8_t)( (c>>0 ) & 0xff ) ) );
#else
	return c;
#endif
}

// hacky utility to merge colors
// fraction=0 => colorA; fraction=1 => colorB; fraction=0.5 => mix
// TODO:  but something's backward in the code or my brain! 
// (let's hope Adafruit_NeoPixel doesn't change its encoding of colors)
uint32_t weighted_average_of_colors( uint32_t colorA, uint32_t colorB,
  float fraction ){
	// TODO:  weight brightness to look more linear to the human eye
	uint8_t RA = (colorA>>16) & 0xff;
	uint8_t GA = (colorA>>8 ) & 0xff;
	uint8_t BA = (colorA>>0 ) & 0xff;
	uint8_t RB = (colorB>>16) & 0xff;
	uint8_t GB = (colorB>>8 ) & 0xff;
	uint8_t BB = (colorB>>0 ) & 0xff;
	return Adafruit_NeoPixel::Color(
	  RA*fraction + RB*(1-fraction),
	  GA*fraction + GB*(1-fraction),
	  BA*fraction + BB*(1-fraction) );
}
