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
#define OFFSETS { -4, -4, -5, -4, -5 }
#define NOISYZERO 0.2  // assume any smaller measurement should be 0
#endif

#if ENABLE_PROTECT
#define PIN_PROTECT 2
#define PROTECT_INTERVAL 0
#define PROTECT_CUTOFF 53.0
#define PROTECT_RECOVER 50.0
#endif

#if ENABLE_INDICATORS
#define PIN_PIXELS {3, 4, 5, 6, 7}
#define NUM_POWER_PIXELS 7  // number LEDs for power
#define NUM_ENERGY_PIXELS 7  // number LEDs for energy
#define NUM_PIXELS (NUM_POWER_PIXELS+NUM_ENERGY_PIXELS)  // number LEDs per bike
#define IND_INTERVAL 500
#define IND_BLINK_INTERVAL 300
#define IND_VOLT_LOW -1
#define IND_VOLT_HIGH 50.0

uint32_t ENERGY_COLORS[] = {
  Adafruit_NeoPixel::Color(0,0,0),
  Adafruit_NeoPixel::Color(255,0,0),
  Adafruit_NeoPixel::Color(0,255,0),
  Adafruit_NeoPixel::Color(255,0,255) };
#define NUM_ENERGY_COLORS (sizeof(ENERGY_COLORS)/sizeof(*ENERGY_COLORS))
// a half-hour of relaxed pedaling ie 60W x 30m * 60s/m in watt seconds
#define ENERGY_PER_SMOOTHIE (60*30*60)

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
int sensorOffset[NUM_AMP_SENSORS] = OFFSETS;

unsigned int voltAdc = 0;
float volts = 0.0;

int ampsRaw[NUM_AMP_SENSORS] = { 0 };
float amps[NUM_AMP_SENSORS] = { 0.0 };
float watts[NUM_AMP_SENSORS] = { 0.0 };
float energy[NUM_AMP_SENSORS] = { 0.0 };  // watt secs
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


////// *********** functions  -  each function is implemented before called, and loop() is at the end, to facilitate using alternative C compilers & IDEs.

//The setup function is called once at startup of the sketch
void setup() {

	Serial.begin(57600);
	Serial.println(VERSION);

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
		strip.setPixelColor(i, r, g, b);
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

//		Serial.print("ind blink ");
//		//Serial.print(indState);
//		//Serial.print(" ");
//		Serial.println(blinkCount++);

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
	doOneRamp(s, 0, NUM_POWER_PIXELS, ledstolight, color, dark);

	// the energy LEDs
	int full_smoothies = energy[s]/ENERGY_PER_SMOOTHIE;
	float partial_smoothie = energy[s] - full_smoothies * ENERGY_PER_SMOOTHIE;
	ledstolight = logEnergyRamp(partial_smoothie);
	if( ledstolight > NUM_ENERGY_PIXELS ) ledstolight=NUM_ENERGY_PIXELS;
	uint32_t curcolor = ENERGY_COLORS[full_smoothies%NUM_ENERGY_COLORS];
	uint32_t nextcolor = ENERGY_COLORS[(full_smoothies+1)%NUM_ENERGY_COLORS];
	// the energy LEDs are upside-down, so subtract ledstolight and swap colors
	doOneRamp(s, NUM_POWER_PIXELS, NUM_ENERGY_PIXELS,
	  NUM_ENERGY_PIXELS-ledstolight, nextcolor, curcolor );

	// and show 'em
	strips[s].show();
}

void doOneRamp(uint8_t s, uint8_t offset, uint8_t num_pixels, float ledstolight, uint32_t firstColor, uint32_t secondColor){
	for( int i=0,pixel=offset; i<num_pixels; i++,pixel++ ){
		uint32_t color =
		  i<(int)ledstolight ?  // definitely firstColor
		    firstColor :
		  i>(int)ledstolight+1 ?  // definitely secondColor
		    secondColor :
		  // else mix the two proportionally
		    weighted_average_of_colors( firstColor, secondColor, ledstolight-(int)ledstolight);
		strips[s].setPixelColor(pixel, color);
	}
}

/* About logPowerRamp and logEnergyRamp:  I love closed form solutions:  they
 * reliably and easily adjust in response to adjusted constants (or to swapping
 * a constant for a variable).  But I don't know how to find a solution for the
 * equation I picked, and I didn't even pick an equation that facilitates
 * twiddling constants (even more so for logEnergyRamp).
 *
 * Maybe I should follow Nio's advice and use splines.  But not yet; let's get
 * this ready to run now.
 */

float logPowerRamp( float p ) {
	/*
	We want an equation of the form:  l = A * log(p-B) - C
	  where l is number of LEDs to light and p is power
	We want ideal data points:
	p = 20W   => l = 0LEDs
	p = 1000W => l = 7LEDs = NUM_POWER_PIXELS
	p = 100W  => l = 3.5LEDs = NUM_POWER_PIXELS/2
	Finding a closed form would be nice but is too tough for me...
	(Can you give me the C expressions?  Or teach me how to find them?)

	So let's just twiddle constants until R draws a nice graph:
	p<-c(20,1000,100); l<-c(0,7,3.5)
	logPowerRamp <- function(p) A*log(p-B)-C
	xmax<-1000; ymax<-8
	pl<-function() {
		plot( logPowerRamp, xlim=c(0,xmax), ylim=c(0,ymax) );
		par(new=TRUE);
		plot( x=p,y=l, xlim=c(0,xmax), ylim=c(0,ymax) );
	}
	A<-1.44; B<-12.5; C<-2.9
	pl()
	logPowerRamp(p)  # 0.00146035 7.02905416 3.53915986
	*/
	float l = 1.44 * log(p-12.5) - 2.9;
	return l<0 ? 0 : l;
}

float logEnergyRamp( float e ) {
	// e is energy measured in watt secs
	/*
	We want an equation of the form:  l = A * log(e-B) - C
	  where l is number of LEDs to light and e is energy
	We want ideal data points:
	e = 0Ws  => l = 0LEDs
	e = ENERGY_PER_SMOOTHIE Ws => l = 7LEDs = NUM_ENERGY_PIXELS
	e = 60W*30s  => l = 1LEDs
	  (give a quick response)
	Finding a closed form would be nice but is too tough for me...
	(Can you give me the C expressions?  Or teach me how to find them?)

	So let's just twiddle constants until R draws a nice graph:
	e<-c(0,60*30*60,60*30); l<-c(0,7,1)
	logEnergyRamp <- function(e) A*log(e-B)-C
	xmax<-60*30*60; ymax<-8
	pl<-function() {
		plot( logEnergyRamp, xlim=c(0,xmax), ylim=c(0,ymax) );
		par(new=TRUE);
		plot( x=e,y=l, xlim=c(0,xmax), ylim=c(0,ymax) );
	}
	A<-0.9; B<--300; C<-5  # a reasonable first approximation
	pl()
	logEnergyRamp(e)
	*/
	float l = 0.9 * log(e+300) - 5;
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
		watts[i] = volts * amps[i];
		totalWatts += watts[i];
		float wattsecs = watts[i] * timeDiffSecs;
		float watthrs = wattsecs / 3600;
		energy[i] += wattsecs; // watt secs
		totalEnergy += watthrs;  // watt hours

	}
	lastEnergy = time;

}


void resetEnergy(int input){
	// if input == -1, reset all
	if(input == -1){
		for(int i=0; i<NUM_AMP_SENSORS; i--){
			energy[i] = 0;
		}
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
		Serial.print(":");
		if(enableRawMode)
			Serial.print(ampsRaw[i]);
		else {
			Serial.print(watts[i], 0);
			Serial.print(",");
			Serial.print(energy[i], 0);
		}
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

// hacky utility to merge colors
// (let's hope Adafruit_NeoPixel doesn't change its encoding of colors)
uint32_t weighted_average_of_colors( uint32_t colorA, uint32_t colorB,
  float fraction ){
	uint8_t RA = (colorA>>16) & 0xff;
	uint8_t GA = (colorA>>8 ) & 0xff;
	uint8_t BA = (colorA>>0 ) & 0xff;
	uint8_t RB = (colorB>>16) & 0xff;
	uint8_t GB = (colorB>>8 ) & 0xff;
	uint8_t BB = (colorB>>0 ) & 0xff;
	return Adafruit_NeoPixel::Color(
	  RA*(1-fraction) + RB*fraction,
	  GA*(1-fraction) + GB*fraction,
	  BA*(1-fraction) + BB*fraction );
}
