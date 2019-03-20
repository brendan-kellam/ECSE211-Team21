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
	private static final int APPROACH_SPEED = 100; //Make sure it's not too fast.

	// Default speed when backing up from can
	private static final int BACKUP_SPEED = -75; //Make sure it's not too slow.

	//Initialize the ultrasonic sensor
	private static final SensorModes usSensor = Vehicle.US_SENSOR; 
	private static SampleProvider usDistance = usSensor.getMode("Distance"); 
	private static float[] usData = new float[usDistance.sampleSize()];
	

	private static int filter = 0;
	private static final int maxfilter = 10;

	//One motor for the light sensor.
	public static final EV3MediumRegulatedMotor lightSensorMotor = Vehicle.FRONT_COLOUR_SENSOR_MOTOR;

	//The colour sensor:

	/**
	 * Color sensor provider
	 */
	private static SampleProvider csProvider = Vehicle.COLOR_SENSOR_FRONT.getRGBMode(); // TODO: this will take long to initialize every time, I'm sure there's a better way.

	/**
	 * Normalized intensity readings from the R, G, B values
	 */
	private static  float[] curRGB = new float[csProvider.sampleSize()];

	/**
	 * Number of values the light sensor will read.
	 */
	private static final int numReadings = 18;
	/**
	 * Sweep angle for the light sensor's motor.
	 */
	private static final int sweepAngle = 180;
	
	/**
	 * How close do we need to get to the can? 
	 * How far do we want to be once the scan terminates? 
	 */
	private static final int approachDistance = 3;
	private static int retreatDistance = 16;

	private static boolean performScan;
	/**
	 * This method will compare the colour of the detect can to the desired colour
	 * @param desiredColour The desired colour to verify
	 * @return true if the can is the desired colour, false otherwise.
	 * @throws InterruptedException 
	 */
	public static boolean checkCanColour(int desiredColour) throws InterruptedException {
		performScan = true;
		approachCan(); //Approach the can
		if (!performScan) {
			return false;
		}
		boolean correctCan = sweepCan(desiredColour);//Check the can colour
		//Beep twice if we found the correct can
		if (correctCan) {
			for (int i=0;i<10;i++) Sound.beep();
		}
		reverseAwayFromCan();
		return correctCan;

	}

	/**
	 * This method will check the colour of a can that it has detected.
	 * @param desiredColour The desired colour to verify
	 * @return true if the can is the desired colour, false otherwise.
	 * @throws InterruptedException 
	 */
	public static void forDemoOneToAnalyzeCansOneAfterEachOther () throws InterruptedException {
		retreatDistance = 8;
		while (true) {
			LCD.clear();
			usSensor.fetchSample(usData,0);	
			int currentDistance = (int) (usData[0] * 100.0);
			LCD.drawString("No object detected", 0, 0);
			while (currentDistance > 15 ) {
				//Read the sensor values.
				usSensor.fetchSample(usData, 0); 
				currentDistance = (int) (usData[0] * 100.0);
			}
			
			LCD.clear();
			LCD.drawString("Object Detected", 0, 0);
			approachCan(); //Approach the can
			printCanColour();//Check the can colour
			reverseAwayFromCan();
			Thread.sleep(3000);
//			Sound.beep();
		}

	}


	/**
	 * Sweep the light sensor around the can. At each interval, compare the detected rgb to the measured rgb of each can.
	 * @param measuredRGB the list of measuredRGB for each can colour.
	 * @param desiredColour the int index of the colour that we want 
	 * @return true if the can we're at is the colour we want.
	 * @throws InterruptedException 
	 */
	private static boolean sweepCan(int desiredColour) throws InterruptedException { 
		//TEMPORARY: TO VISUALIZE RESULTS.
		LCD.clear();
		LCD.drawString("Red", 0, 0);
		LCD.drawString("Green", 0, 1);
		LCD.drawString("Blue", 0, 2);
		//REMOVE^

		int colours[] = new int[numReadings];
		float redAvg = 0;
		float greenAvg = 0;
		float blueAvg = 0;

		for (int sample=0;sample<numReadings;sample++) {

			csProvider.fetchSample(curRGB, 0);
			//Thread.sleep(25); //May or may not be necessary.

			float red = curRGB[0] * 1000; 
			float green = curRGB[1] * 1000;
			float blue =  curRGB[2] * 1000;

			//Just to log the data. Can remove from final results.
			redAvg += curRGB[0] * 1000; 
			greenAvg += curRGB[1] * 1000; 
			blueAvg += curRGB[2] * 1000; 

			/////////////// Purely for display purposes //////////////
			LCD.drawInt((int) red, 6, 0);
			LCD.drawInt((int) green, 6, 1);
			LCD.drawInt((int) blue, 6, 2);
			Log.log(Sender.colourDetection, "red = " + red + " | green = " + green + " | bluee = " + blue);
			//////////////////////////////////////////////////////////

			//Determine the colour of the sample, hold it in the array.
			colours[sample] = determineColourForSample(red,green,blue);

			//Rotate to the next sample if there is one.
			lightSensorMotor.rotate(sweepAngle/numReadings);
			Thread.sleep(50);
		}

		//Just to log the data:
		redAvg = redAvg/numReadings;
		greenAvg = greenAvg/numReadings;
		blueAvg = blueAvg/numReadings;

		////////////////////////////////
		//THESE LCD ARE JUST FOR DISPLAY AND TESTING. Can remove from final.
		LCD.clear();
		LCD.drawString("RedA: ", 0, 0);
		LCD.drawString("GreenA: ", 0, 1);
		LCD.drawString("BlueA: ", 0, 2);
		LCD.drawInt((int) redAvg, 6, 0);
		LCD.drawInt((int) greenAvg, 6, 1);
		LCD.drawInt((int) blueAvg, 6, 2);
		Log.log(Sender.colourDetection, "red Average = " + redAvg + " | green Average = " + greenAvg + " | blue Average = " + blueAvg);
		////////////////////////////////

		//Bring the sensor back to its original.
		lightSensorMotor.rotate(-sweepAngle);

		//Determine the most common colour from the samples.
		int mostCommonColour = countColours(colours);
		String canColour = convertColourIndexToWord(mostCommonColour); 
		LCD.drawString("The can colour is: ",0,5);
		LCD.drawString(canColour, 2, 6);
		return (mostCommonColour == desiredColour); 
	}

	/**
	 * Takes as index an integer value and outputs the corresponding colour
	 * @param colour Index value 
	 * @return
	 */
	private static String convertColourIndexToWord(int colour) {

		if (colour == 1) {
			return "Blue";
		}
		else if (colour == 2) {
			return "Green";
		}
		else if (colour == 3) {
			return "Yellow";
		}
		else if (colour == 4) {
			return "Red";
		}
		else {
			return ":("; //This can't ever happen.
		}
	}

	/**
	 * Determine the colour of a sample
	 * @param red The R value from the RGB sample
	 * @param green The G value from the RGB sample
	 * @param blue The B value from the RGB sample
	 * @return the index value of the colour that it detected from the sample(is the sample red,green,yellow, or blue)
	 */
	private static int determineColourForSample(float red,float green, float blue) {
		//Index 0 is uncertain
		//Index 1 is blue
		//Index 2 is green
		//Index 3 is yellow
		//Index 4 is red
		if ((blue > red) && (blue > green)) { //Blue can
			return 1;
		}
		if ( (green > red) && (green - blue > 5) ) {//We're green
			return 2;
		}
		if ((red > green) && (green - blue > 15)) { // We're yellow
			return 3;
		}
		if ( (red - green > 12) && (red - blue > 15) && (green - blue < 15)) { // Red can
			return 4;
		}

		return 0;
	}


	/**
	 * Counts the number of colours from the list of samples
	 * @param colourSamples 
	 * @return the colour that occurs most often from the sample.
	 */
	private static int countColours(int[] colourSamples) {
		int[] occurrenceOfColour = {0,0,0,0,0};

		for (int i=0;i<colourSamples.length;i++) {
			occurrenceOfColour[colourSamples[i]]++;
		}

		return mostCommon(occurrenceOfColour);
	}

	/**
	 * 
	 * @param occurrenceOfColour the number of times each colour occurs in the samples.
	 * @return the index of the colour with the most occurences
	 */
	private static int mostCommon(int[] occurrenceOfColour) {
		int max = -1,colourIndex = -1;

		//Start from 1 cause we don't care about the values with index 0(uncertain values)
		for (int i=1;i<occurrenceOfColour.length;i++) {
			if ( occurrenceOfColour[i] > max) {
				max = occurrenceOfColour[i];
				colourIndex = i;
			}
		}
		return colourIndex;
	}

	/**
	 * Approach the can within the specified distance "approachDistance"
	 */
	private static void approachCan() {	

		usSensor.fetchSample(usData,0);	
		int currentDistance = (int) (usData[0] * 100.0);

		long startTime = System.currentTimeMillis();
		// Drive forward slowly.
		Vehicle.setMotorSpeeds(APPROACH_SPEED, APPROACH_SPEED); 
		//Drive forward slowly.
		while (currentDistance > approachDistance  && performScan) {
			//Read the sensor values.
			usSensor.fetchSample(usData, 0); 
			currentDistance = (int) (usData[0] * 100.0);

			//FAULT TOLERANCE:
			long currTime = System.currentTimeMillis();
			if (currTime - startTime > 4000) { //If we travel for 4 seconds and haven't found a can, end the routine
				performScan = false;
			}
			
			//Put the thread to sleep
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		//Stop the car
//		Sound.beep();
		Vehicle.setMotorSpeeds(0, 0);
	}
	
	/**
	 * Reverse away from the can so that we have space to pass it on the next go.
	 * @throws InterruptedException 
	 */
	private static void reverseAwayFromCan() throws InterruptedException {

		// Drive forward slowly.
		Vehicle.setMotorSpeeds(BACKUP_SPEED, BACKUP_SPEED);
		Thread.sleep(250);
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
	 * Determine the colour of the can being swept, and print it to the LCD.
	 * @throws InterruptedException
	 */
	private static void printCanColour() throws InterruptedException { 

		int colours[] = new int[numReadings];
		float redAvg = 0;
		float greenAvg = 0;
		float blueAvg = 0;
		for (int sample=0;sample<numReadings;sample++) {

			csProvider.fetchSample(curRGB, 0);
			//Thread.sleep(25); //May or may not be necessary.

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
			lightSensorMotor.rotate(sweepAngle/numReadings);
		}

		//Just to log the data:
		redAvg = redAvg/numReadings;
		greenAvg = greenAvg/numReadings;
		blueAvg = blueAvg/numReadings;

		////////////////////////////////
		//THESE LCD ARE JUST FOR DISPLAY AND TESTING. Can remove from final.
		LCD.clear();
		LCD.drawString("Red: " + redAvg, 0, 0);
		LCD.drawString("Green: " + greenAvg, 0, 1);
		LCD.drawString("Blue: " + blueAvg, 0, 2);
		Log.log(Sender.colourDetection, "red Average = " + redAvg + " | green Average = " + greenAvg + " | blue Average = " + blueAvg);
		////////////////////////////////

		//Bring the sensor back to its original.
		lightSensorMotor.rotate(-sweepAngle);

		//Determine the most common colour from the samples.
		int mostCommonColour = countColours(colours);
		String canColour = convertColourIndexToWord(mostCommonColour); 
		LCD.drawString("The can colour is: " ,0,5);
		LCD.drawString(canColour, 2, 6);
	}

}
