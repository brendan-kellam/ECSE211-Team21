package ca.mcgill.ecse211.colour;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.util.Log.Sender;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

//Sweep twice, one for each side. Sum, then average.

public class ColourDetection {

	// Default speed when approaching can
	private final int APPROACH_SPEED = 120; //Make sure it's not too fast.

	// Default speed when backing up from can
	private final int BACKUP_SPEED = -75; //Make sure it's not too slow.

	/*
	 * Initialize the ultrasonic sensor
	 */
	private static final SensorModes usSensor = Vehicle.US_SENSOR; 
	private static SampleProvider usDistance = usSensor.getMode("Distance"); 
	private static float[] usData = new float[usDistance.sampleSize()];

	/*
	 * TODO: Consider implementing a filter when advancing:
	 */
	private final int filter = 0;
	private final int maxfilter = 10;

	/**
	 * Colour sensor motor
	 */
	//One motor for the light sensor.
	public final EV3MediumRegulatedMotor colourSensorMotor = Vehicle.FRONT_COLOUR_SENSOR_MOTOR;

	/**
	 * Color sensor provider
	 */
	private final SampleProvider csProvider = Vehicle.COLOR_SENSOR_FRONT.getRGBMode();

	/**
	 * Normalized intensity readings from the R, G, B values
	 */
	private float[] curRGB = new float[csProvider.sampleSize()];

	/**
	 * Number of values the light sensor will read.
	 */
	private final int numReadings = 18;

	/**
	 * Sweep angle for the light sensor's motor.
	 */
	private final int sweepAngle = 180;

	/**
	 * How close do we need to get to the can? 
	 * How far do we want to be once the scan terminates? 
	 */
	private final int approachDistance = 3;
	private int retreatDistance = 12; // This may be modified in the test


	/*
	 * We'll have to convert the colour integer value to a COLOUR NAME
	 */
	private enum Colour {
		RED,
		BLUE,
		GREEN,
		YELLOW
	}

	private Colour desiredColour;

	/*
	 * If we drive too long, we no longer wnat to perform a scan
	 */
	private boolean performScan;

	/*
	 * Constructor for the colour detection; make the desired colour
	 */
	public ColourDetection(int desiredIntegerOfCan) {
		this.desiredColour = colourValueOf(desiredIntegerOfCan);
	}

	/**
	 * This method will compare the colour of the detect can to the desired colour
	 * @param desiredColour The desired colour to verify
	 * @return true if the can is the desired colour, false otherwise.
	 * @throws InterruptedException 
	 */
	public boolean checkCanColour() {
		this.performScan = true;
		approachCan(); //Approach the can

		if (!performScan) {
			return false;
		}

		boolean correctCan = sweepCan(this.desiredColour);

		//Beep twice if we found the correct can
		if (correctCan) {
			for (int i=0;i<10;i++) Sound.beep();
		}

		reverseAwayFromCan();
		return correctCan;

	}


	/**
	 * Sweep the light sensor around the can. At each interval, compare the detected rgb to the measured rgb of each can.
	 * @param measuredRGB the list of measuredRGB for each can colour.
	 * @param desiredColour the int index of the colour that we want 
	 * @return true if the can we're at is the colour we want.
	 */
	private boolean sweepCan(Colour desiredColour) { 

		Colour colours[] = new Colour[numReadings]; // Create an array of the colour samples

		for (int sample=0;sample<numReadings;sample++) {

			csProvider.fetchSample(curRGB, 0);

			float red = curRGB[0] * 1000; 
			float green = curRGB[1] * 1000;
			float blue =  curRGB[2] * 1000;

			//Determine the colour of the sample, hold it in the array.
			colours[sample] = determineColourForSample(red,green,blue);

			//Rotate to the next sample if there is one.
			colourSensorMotor.rotate(sweepAngle/numReadings);
			sleep(50);
		}

		//Bring the sensor back to its original position
		colourSensorMotor.rotate(-sweepAngle);
		return (mostCommonColour(colours) == desiredColour); 
	}

	/**
	 * Determine the colour of a sample
	 * @param red The R value from the RGB sample
	 * @param green The G value from the RGB sample
	 * @param blue The B value from the RGB sample
	 * @return the index value of the colour that it detected from the sample(is the sample red,green,yellow, or blue)
	 */
	private Colour determineColourForSample(float red,float green, float blue) {
		if ((blue > red) && (blue > green)) { //Blue can
			return Colour.BLUE;
		}
		if ((green > red) && (green - blue > 5)) {//We're green
			return Colour.GREEN;
		}
		if ((red > green) && (green - blue > 15)) { // We're yellow
			return Colour.YELLOW;
		}
		if ( (red - green > 12) && (red - blue > 15) && (green - blue < 15)) { // Red can
			return Colour.RED;
		}

		return Colour.BLUE;
	}


	/**
	 * Counts the number of colours from the list of samples
	 * @param colourSamples 
	 * @return the colour that occurs most often from the sample.
	 */
	private int[] countColours(Colour[] colourSamples) {

		int[] occurrenceOfColour = {0,0,0,0,0};

		for (int i=0;i<colourSamples.length;i++) {
			switch (colourSamples[i]) {
			case BLUE:
			{
				occurrenceOfColour[1]++;
				break;
			}
			case GREEN:
			{
				occurrenceOfColour[2]++;
				break;
			}
			case YELLOW:
			{
				occurrenceOfColour[3]++;
				break;
			}
			case RED:
			{
				occurrenceOfColour[4]++;
				break;
			}	
			}
		}
		return occurrenceOfColour;
	}

	/**
	 * 
	 * @param occurrenceOfColour the number of times each colour occurs in the samples.
	 * @return the index of the colour with the most occurences
	 */
	private Colour mostCommonColour(Colour[] colourSamples) {
		int max = -1,colourIndex = -1;
		int[] occurrenceOfColour = countColours(colourSamples);
		//Start from 1 cause we've only considered cans being of index 1,2,3,4
		for (int i=1;i<occurrenceOfColour.length;i++) {
			if ( occurrenceOfColour[i] > max) {
				max = occurrenceOfColour[i];
				colourIndex = i; 
				//Set the colour index at the point in the array with the most cans of a specific colour
			}
		}
		return colourValueOf(colourIndex);
	}

	/**
	 * Approach the can within the specified distance "approachDistance"
	 */
	private void approachCan() {	

		usSensor.fetchSample(usData,0);	
		int currentDistance = (int) (usData[0] * 100.0);

		long startTime = System.currentTimeMillis();
		// Drive forward slowly.
		Vehicle.setMotorSpeeds(APPROACH_SPEED, APPROACH_SPEED); 

		this.performScan = true;
		while (currentDistance > approachDistance  && performScan) {
			//Read the sensor values.
			usSensor.fetchSample(usData, 0); 
			currentDistance = (int) (usData[0] * 100.0);

			//FAULT TOLERANCE:
			long currTime = System.currentTimeMillis();
			if (currTime - startTime > 4000) { //If we travel for 4 seconds and haven't found a can, end the routine
				this.performScan = false;
			}
			sleep(20);
		}

		//Stop the car
		Vehicle.setMotorSpeeds(0, 0);
	}



	/**
	 * Reverse away from the can so that we have space to pass it on the next go.
	 * @throws InterruptedException 
	 */
	private void reverseAwayFromCan() {

		// Back away
		Vehicle.setMotorSpeeds(BACKUP_SPEED, BACKUP_SPEED);
		sleep(250);
		usSensor.fetchSample(usData,0);	
		int currentDistance = (int) (usData[0] * 100.0);
		//Retreat until we have enough space. 
		while (currentDistance < retreatDistance ) {
			//Read the sensor values.
			usSensor.fetchSample(usData, 0); 
			currentDistance = (int) (usData[0] * 100.0);
		}

		//Stop the car
		Vehicle.setMotorSpeeds(0, 0);

	}

	/**
	 * Convert an integer index into a desired colour value
	 * @param desiredIntegerOfCan
	 * @return
	 */
	private Colour colourValueOf(int desiredIntegerOfCan) {

		switch (desiredIntegerOfCan) {
		case (1):
		{
			return Colour.BLUE;	
		}
		case (2):
		{
			return Colour.GREEN;
		}
		case (3):
		{
			return Colour.YELLOW	;
		}
		case (4):
		{
			return Colour.RED;
		}
		default:
			return Colour.BLUE;
		}
	}

	/*
	 * Just to make the other methods a little lighter.
	 */
	private void sleep(int wait) {
		//Put the thread to sleep
		try {
			Thread.sleep(wait);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	/////////////////////FOR TESTING PURPOSES///////////////////////////////////////
	/**
	 * This method will check the colour of a can that it has detected.
	 * @param desiredColour The desired colour to verify
	 * @return true if the can is the desired colour, false otherwise.
	 * @throws InterruptedException 
	 */
	public void testCanColours () {
		retreatDistance = 8;
		while (true) {
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
				LCD.clear();
				LCD.drawString("Object Detected", 0, 0);
				approachCan(); //Approach the can
				testSweep();//Check the can colour
				reverseAwayFromCan();
				Sound.beep();
		}

	}

	/**
	 * Determine the colour of the can being swept, and print it to the LCD.
	 */
	private void testSweep() { 

		Colour colours[] = new Colour[numReadings];
		float redAvg = 0;
		float greenAvg = 0;
		float blueAvg = 0;
		for (int sample=0;sample<numReadings;sample++) {

			csProvider.fetchSample(curRGB, 0);

			float red = curRGB[0] * 1000; 
			float green = curRGB[1] * 1000;
			float blue =  curRGB[2] * 1000;

			//Just to log the data. Can remove from final results.
			redAvg += curRGB[0] * 1000; 
			greenAvg += curRGB[1] * 1000; 
			blueAvg += curRGB[2] * 1000; 

			/////////////// Purely for logging purposes //////////////
			Log.log(Sender.colourDetection, "red = " + red + " | green = " + green + " | blue = " + blue);
			//////////////////////////////////////////////////////////

			//Determine the colour of the sample, hold it in the array.
			colours[sample] = determineColourForSample(red,green,blue);

			//Rotate to the next sample if there is one.
			colourSensorMotor.rotate(sweepAngle/numReadings);
		}

		//Just to log the data:
		redAvg = redAvg/numReadings;
		greenAvg = greenAvg/numReadings;
		blueAvg = blueAvg/numReadings;

		//////////////////////////////////////////////////////////
		LCD.clear();
		LCD.drawString("Red: " + redAvg, 0, 0);
		LCD.drawString("Green: " + greenAvg, 0, 1);
		LCD.drawString("Blue: " + blueAvg, 0, 2);
		Log.log(Sender.colourDetection, "red Average = " + redAvg + " | green Average = " + greenAvg + " | blue Average = " + blueAvg);
		//////////////////////////////////////////////////////////

		//Bring the sensor back to its original.
		colourSensorMotor.rotate(-sweepAngle);

		//Determine the most common colour from the samples.
		Colour mostCommonColour = mostCommonColour(colours);
		LCD.drawString("The can colour is: " ,0,5);
		LCD.drawString(mostCommonColour.toString(), 2, 6);
	}

}
