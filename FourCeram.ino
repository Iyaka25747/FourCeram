//Version V2: Improved with new algorithm
#include <TM1637Display.h> // 7 segments display library
#include <PID_v1.h> // PID Library
#define PIN_INPUT 0
#define PIN_OUTPUT 1
#define LOW 0
#define HIGH 1
#define RELAY_PIN 6 // Command of relay

/*
PUBLIC VARIABLES
*/
//Temperature curve each point contains; delta_celesius, DeltaTime}. Delta_celesius[Celesius] = the temperature difference and DeltaTime[minute] is the duration.
// E.g. 230, 60 is a ramp of +230CÂ° in 60 minutes, or 0,180 is a stable temperature during 3hours.
//PRODUCTION values
const int segmentDeltaCelesius[] = { 230, 500, 0, -80, 0, -650 };
const int segmentDurationMinutes[] = { 60, 120, 180, 60, 300, 300 };
const int segmentTotalNumber = 6; // the curve above contains 6 points
////TESTING values oven not connected
//const int segmentDeltaCelesius[] = { 230, 0, -230, 0 };
//const int segmentDurationMinutes[] = { 1, 1, 1, 1 };
//const int segmentTotalNumber = 4; // the curve above contains 6 points
////TESTING values oven connected
//const int segmentDeltaCelesius[] = {230,0,-230};
//const int segmentDurationMinutes[] = {10,10,10};
//const int segmentTotalNumber = 3; // the curve above contains 6 points
/*Countdown timer before we start the heating program*/
float countdownTimeMinute = 0.16;
//int countdownTimeMillis = countdownTimeMinute * 60000;
int countdownTimeMillis = 10000;
int initialtemperature = 20; // Used if the oven is not cold during at start time.

/*
PRIVATE variable
*/

/*
Debug
*/
boolean debugCode = true;

/*
HEX Display, measure and temperature setpoint
*/
//Measured temperature
const int CLK = 3; //Set the CLK pin connection to the display
const int DIO1 = 2; //Set the DIO pin connection to the display
TM1637Display displayMeasuredTemp(CLK, DIO1); //set up the 4-Digit Display1.

//Setpoint
// Let's use the CLK const above for both display
const int DIO2 = 4; //Set the DIO pin connection to the display
TM1637Display displaySetTemp(CLK, DIO2); //set up the 4-Digit Display2.
long lastDisplayTime; // Used for time interval between display refresh


/*
Temperature management values
*/
int numTemperature = 0; // the computed temperature in CÂ°
long setPointCelesius;//Setpoint in Celesius

// Oven sensor & relay values
const int analogInPin = A0; // Analog input pin temperature signal from op amp of the oven 0-5 V
int sensorDigit = 0;        // value read from the analog input
long sensorCelesius = 0; // value mapped from sensor value, use for output to the PWM (analog out)
boolean relayState = LOW; // oven relay value HIGH or LOW use to open or close the current to the oven


// Smoothing input value parameters to get rid of erratic values
// Define the number of samples to keep track of.  The higher the number,
// the more the readings will be smoothed, but the slower the output will
// respond to the input.
const int numReadings = 50;  // number of samples for averaging
int readings[numReadings]; // array use to hold the the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total, total of the array
int average = 0;                // the computed average


/*
Curve management values
*/
int currentSegment; // used for the partial line (segment) of the curve
int segmentDeltaTemp;
int segmentTotalNumberOfSteps;
int segmentDeltaTempCelesius;
//	int stepInterval = 2000; // DEBUG we will break the segment in 2 seconds steps.
	int stepInterval = 20000; // we will break the segment in steps during 'stepInterval' [sec]

//TEMPERATURE MAPPING
// Temperature between 0 - 100CÂ° are not displayed correctly

// Transfer function to map singal board in into CÂ°
// f(x)=  a*x + d; f = temperature [CÂ°]; x = board in
float tempFn_slope = 1.62;
float tempFn_origine = -44.44;

// PID values
//Define Variables we'll be connecting to
double setPoint, PidInput, PidOutput;

//Define the aggressive and conservative Tuning Parameters
double aggKp = 500, aggKi = 0, aggKd = 0;
//double consKp=1, consKi=0.05, consKd=0.25;
//Specify the links and initial tuning parameters
double consKp = 120, consKi = 0.666, consKd = 0; // PID Parameters to tune
// Use this value to set switching from aggressive to conservative mode. The value is defined by the delta between the value when you set oven relay to OFF (heating switch off) and the value of the summit curve value
int aggressiveLimit = 60;
boolean aggressiveMode = false;
PID myPID(&PidInput, &PidOutput, &setPoint, consKp, consKi, consKd, DIRECT);

int WindowSize = 5000; //window size for PWM to the relay
unsigned long windowStartTime; // private to PID

// MONITORING
// Set Monitoring output type: Readable directly or Copy past CSV to table (excel...)
int monitor_CSV = 0; // 0 = No log, 1 = csv log copy paste, 2 = serial monitor
//int logPeriod = 5000; // DEBUG 5 second. Interval in milli second between records, write a new line every "monitor_period" millisecondes
int logPeriod = 20000; // 5 second. Interval in milli second between records, write a new line every "monitor_period" millisecondes
unsigned long lastMonitorStartPeriod = millis(); // store the start time of the new line
//STORE DATA VALUE in array
const int timeIndex = 0;
const int tempSignalInIndex = 1;
const int PID_computedIndex = 2;
const int setPointIndex = 3;
const int tempCelesiusIndex = 4;
const int segmentNbIndex = 5;
const int numberOfDataRecords = 6;
char* dataLabel[] = { "time_Millis", "read_in", "PID_out", "setPoint",
		"temp_celesius", "SegmentNb" };
long logData[numberOfDataRecords];

void setup() {

	pinMode(RELAY_PIN, PIN_OUTPUT);	// set relay pin to output
	displayMeasuredTemp.setBrightness(0x0a);//set the display to maximum brightness
	displaySetTemp.setBrightness(0x0a);  //set the display to maximum brightness
	Serial.begin(9600);			// initialize serial communications at 9600 bps:

	//PID setup
	windowStartTime = millis();
	//SETPOINT  150 = 198 CÂ°, 220-->311 CÂ°, 274-->399 CÂ°, 398-->600 CÂ°,583-->900, 645-->1000CÂ°,
	//  setPoint = 583;
	//  storeDataValue[setPointIndex] = setPoint;
	//tell the PID to range between 0 and the full window size
	myPID.SetOutputLimits(0, WindowSize);
	//turn the PID on
	myPID.SetMode(AUTOMATIC);
	//****** PID END   **************

	// Let's initialize  the smooting array to 0:
	for (int thisReading = 0; thisReading < numReadings; thisReading++) {
		readings[thisReading] = 0;
	}
	// SETUP Log, Monitoring
	if (monitor_CSV == 2) {
		Serial.println(" +++++ HERE WE GO ... ++++");
	}
	if (monitor_CSV == 1) // initialize column header
			{
		//Print column label header
		for (int i = 0; i < numberOfDataRecords; i++) {
			Serial.print("\t");
			Serial.print(dataLabel[i]);
		}
		Serial.println();
	}

	int ovenStartTemp = readAverageInput();
	int ovenStartTempCelesius = mapTempDigitToCelesius(ovenStartTemp);
	myPID.SetTunings(consKp, consKi, consKd);
	setPoint = mapTempCelesiusToDigit(20);//Define initial setpoint. Supposed to be at 20C° if oven is cold
	setPointCelesius = 20;
	int stepIncrementCelesius;

	if (debugCode) {
		Serial.println("Let's cook something");
		Serial.print("Initial oven Digit/C: ");
		Serial.print(ovenStartTemp);
		Serial.print(" / ");
		Serial.println(ovenStartTempCelesius);
		Serial.print("Segment step duration [millis]:");
		Serial.println(stepInterval);
		Serial.print("Starting setpoint 20C in digit: ");
		Serial.println(setPoint);
		Serial.println();

	}

}
// SETUP END ///

void loop() {




	countDownTimer(countdownTimeMillis); // start the countdown timer before we start
	////Iteration through curves
	for (currentSegment = 0; currentSegment <= segmentTotalNumber; currentSegment++) {
		//int stepDuration = 300000; // we will break the segment in 5minutes steps.
		logData[segmentNbIndex] = currentSegment;
		segmentTotalNumberOfSteps = segmentDurationMinutes[currentSegment] * 60000 / stepInterval; // number of steps in the segment
		segmentDeltaTempCelesius = segmentDeltaCelesius[currentSegment];
//		if (currentSegment==0) { // Remove current oven temperature before computing deltas
//			if (debugCode) {
//				Serial.print("Adjusting first segment delta temp, segmentDeltaTempCelesius before:");
//				Serial.print(segmentDeltaTempCelesius);
//				Serial.print(" - ");
//				Serial.println(ovenStartTempCelesius);
//				segmentDeltaTempCelesius = segmentDeltaTempCelesius - ovenStartTempCelesius;
//			}
//		}
//		if (currentSegment == segmentTotalNumber){
//			if (debugCode) {
//				Serial.print("Adjusting last segment delta temp, segmentDeltaTempCelesius before:");
//				Serial.print(segmentDeltaTempCelesius);
//				Serial.print(" - ");
//				Serial.println(ovenStartTempCelesius);
//				segmentDeltaTempCelesius = segmentDeltaTempCelesius - ovenStartTempCelesius;
//				segmentDeltaTemp = mapTempCelesiusToDigit(segmentDeltaTempCelesius);
//			}
//		}
		float segmentDeltaTempDigit = (segmentDeltaTempCelesius / tempFn_slope);
		float stepDeltaTempDigit = segmentDeltaTempDigit / segmentTotalNumberOfSteps;
		if (debugCode) {
			Serial.print("++Starting segment: ");
			Serial.print(currentSegment+1);
			Serial.print("/");
			Serial.println(segmentTotalNumber);
			//TEMP C°
			Serial.print("\tSETPOINT C°- currentC°:");
			Serial.print(setPointCelesius);
			Serial.print("\t, DeltaC°:");
			Serial.print(segmentDeltaCelesius[currentSegment]);
			Serial.print("\t, targetC° (current+delta):");
			Serial.println(setPointCelesius + segmentDeltaCelesius[currentSegment]);
			//TEMP Digit
			Serial.print("\tSETPOINT DIGIT- current:");
			Serial.print(setPoint);
			Serial.print("\t, Delta:");
			Serial.print(mapTempCelesiusToDigit(segmentDeltaCelesius[currentSegment]));
			Serial.print("\t, target (current+delta):");
			Serial.println(setPoint + mapTempCelesiusToDigit(segmentDeltaCelesius[currentSegment]));

			Serial.print("\tsegmentDurationMinutes:");
			Serial.println(segmentDurationMinutes[currentSegment]);
			Serial.print("\tSegment increment C°/digit:");
			Serial.print(segmentDeltaTempCelesius);
			Serial.print("/");
			Serial.println(segmentDeltaTempDigit);
			Serial.print("\tSTEP in segment, total Nb:");
			Serial.println(segmentTotalNumberOfSteps);
			Serial.print("\tSTEP in segment, stepDeltaTempDigit:");
			Serial.println(stepDeltaTempDigit);
			Serial.println();
		}

		if (debugCode) {
			Serial.println("Starting steps in segment");
		}
		for (int i = 0; i <= segmentTotalNumberOfSteps; i++) {
			//DISPLAY2 SETPOINT in Celesius 7 Segment
			setPointCelesius = mapTempDigitToCelesius(setPoint);
			displaySetTemp.showNumberDec(setPointCelesius); // Set Temperature display
			if (debugCode) {
				Serial.print("Step: ");
				Serial.print(i+1);
				Serial.print(" / ");
				Serial.println(segmentTotalNumberOfSteps);
				Serial.print("Setpoint C°/Digit: ");
				Serial.print(setPointCelesius);
				Serial.print(" / ");
				Serial.println(setPoint);
				Serial.print("Sensor C°/Digit: ");
				Serial.print(sensorCelesius);
				Serial.print(" / ");
				Serial.println(sensorDigit);
				Serial.println();
			}

			long startIteration = millis();
			while (millis() - startIteration < stepInterval) {
				//void runTempSegment (int steps; int incrementTemp){
				logData[timeIndex] = millis(); // Let's store the start time of the loop beginning
				sensorDigit = readAverageInput();
				logData[tempSignalInIndex] = sensorDigit; // Let's store the measured average in
				//sensorInCelesius = map(sensorValue, 14, 1023, 25, 1596);
				sensorCelesius = mapTempDigitToCelesius(sensorDigit);
				logData[tempCelesiusIndex] = sensorCelesius; // Let's store the measured average temperature
				//				  Serial.print("Temperature in C: ");
				//				  Serial.println(sensorCelesius);
				// Display the value on 7 segment display1 every 1 second
				if (millis() - 1000 > lastDisplayTime) {
					int displayedValueCelesius = sensorCelesius;
					if (displayedValueCelesius < 80) {
						displayedValueCelesius = 80; //Let's start display at 80C°, before 100C° the sensor it's not accurate
					}
					displayMeasuredTemp.showNumberDec(displayedValueCelesius); //Display the numCounter value;
					lastDisplayTime = millis(); // Let's display the value every second
				}
				//**** PID START ****
				PidInput = sensorDigit;
				myPID.Compute();
				logData[PID_computedIndex] = PidOutput;
				/************************************************
				 * turn the pid output in a timewindows on/off -  output/relay
				 ************************************************/
				if (millis() - windowStartTime > WindowSize) { //time to shift the Relay Window
					windowStartTime += WindowSize;
				}
				if (PidOutput < millis() - windowStartTime) // Set Relay pin HIGH/LOW
						{
					relayState = LOW;
					digitalWrite(RELAY_PIN, relayState);
				} else {
					relayState = HIGH;
					digitalWrite(RELAY_PIN, relayState);
				}
				//**** PID END   ****
				// Monitoring output with data recorded
				if (millis() - lastMonitorStartPeriod > logPeriod) {
					if (monitor_CSV == 2) {
						// print the results to the serial monitor:
						Serial.print("Time=");
						Serial.print(millis());
						Serial.print("\tBoard signal in= ");
						Serial.print(sensorDigit);
						Serial.print("\tPID in= ");
						Serial.print(PidInput);
						Serial.print("\tPID out computed= ");
						Serial.print(PidOutput);
						Serial.print("\tsetPoint=");
						Serial.print(setPoint);
						//Serial.print("\tMapped sensor (temp.)= ");
						//Serial.print(sensorInCelesius);
						//Serial.print("\tRelay= ");
						//Serial.print(relayState);
						//Serial.print("\t(mil-winStrtTm)value= ");
						//Serial.println(millis() - windowStartTime);
						Serial.print("\taggressive mode= ");
						Serial.println(aggressiveMode);

					}

					if (monitor_CSV == 1) {
						//Print the stored values
						for (int i = 0; i < numberOfDataRecords; i++) {
//							Serial.print("SegmentNB:");
//							Serial.println(logData[segmentNbIndex]);
							Serial.print("\t");
							Serial.print(logData[i]);
						}
						Serial.println();
					}
					lastMonitorStartPeriod = millis();
				}
			}
			setPoint = setPoint + stepDeltaTempDigit;
			setPointCelesius = mapTempDigitToCelesius(setPoint);
			logData[setPointIndex] = setPoint; //record new setPoint
//			if (debugCode) {
//				Serial.print("New setPoint:");
//				Serial.println(setPoint);
//			}
		}
	}

	Serial.println("+++THE END+++");
	while (true) {
		if (millis() - 1000 > lastDisplayTime) {
			lastDisplayTime = millis(); // Let's display the value every second
			sensorDigit = readAverageInput();
			sensorCelesius = mapTempDigitToCelesius(sensorDigit);
			int displayedValueCelesius = sensorCelesius;
			if (displayedValueCelesius < 80) {
				displayedValueCelesius = 80; //Let's start display at 80C°, before 100C° the sensor it's not accurate
			}
			displayMeasuredTemp.showNumberDec(displayedValueCelesius); //Display the numCounter value;

			setPointCelesius = mapTempDigitToCelesius(setPoint);
			displaySetTemp.showNumberDec(setPointCelesius); // Set Temperature display

		}
	}
}

int readAverageInput() {
	// Let's read some input values and compute the average to get rid of erratic values
	for (int thisReading = 0; thisReading < numReadings; thisReading++) {
		// subtract the last reading:
		total = total - readings[readIndex];
		// read from the sensor:
		readings[readIndex] = analogRead(analogInPin);
		// add the reading to the total:
		total = total + readings[readIndex];
		// advance to the next position in the array:
		readIndex = readIndex + 1;

		// if we're at the end of the array...
		if (readIndex >= numReadings) {
			// ...wrap around to the beginning:
			readIndex = 0;
		}
	}
	// calculate the average:
	average = total / numReadings;
	// send it to the computer as ASCII digits
	// Serial.println(average);
	//  delay(1);        // delay in between reads for stability
	return average;
}

float mapTempDigitToCelesius(int singal_in) {
	float result;
	result = tempFn_slope * singal_in + tempFn_origine;
	return result;
}

float mapTempCelesiusToDigit(float temp_in) {
	float result;
	result = (temp_in - tempFn_origine) / tempFn_slope;
	return result;
}

void countDownTimer(int timeSetMillis) {
	int timeSetMinute = timeSetMillis / 60000;
	unsigned long startTimeCounter = millis();
	unsigned long refreshInterval = 500;
	unsigned long lastCounterDisplayMillis = millis();
	unsigned long counterDurationMillis = 0;
	unsigned long currentMillis;
	if (debugCode) {
		Serial.print("Starting Countdown for [minute]: ");
		Serial.println(timeSetMinute);
	}
	do {
		counterDurationMillis = (millis() - startTimeCounter);
		//		Serial.print("Counter duration [millis]: ");
		//		Serial.println(counterDurationMillis);
		currentMillis = millis();
		//    Serial.print("currentMillis: ");
		//    Serial.print(currentMillis);
		//    Serial.print("\tLast counter display time: ");
		//    Serial.print(lastCounterDisplayMillis);
		//    Serial.print("\tdif: ");
		//    Serial.println(currentMillis - lastCounterDisplayMillis);
		if (currentMillis - lastCounterDisplayMillis >= refreshInterval) { //refresh every 1second
			//			display2.showNumberDec(timeSetMinute-(counterDurationMillis/60000)); //Display the numCounter value in minutes
			displaySetTemp.showNumberDec(
					(timeSetMillis - counterDurationMillis) / 1000); //Display the numCounter value in second
			//			Serial.println(currentMillis - lastCounterDisplayMillis);
			lastCounterDisplayMillis = currentMillis;
			//			Serial.println("Display refreshed");
		}
		//   Serial.print("Delta: ");
		//   Serial.print(currentMillis - startTimeCounter);
		//   Serial.print("\ttimeSetMillis: ");
		//   Serial.println(timeSetMillis);
	} while (currentMillis - startTimeCounter <= timeSetMillis);
	if (debugCode) {
		Serial.println("Exiting countdown timer");
	}
}
