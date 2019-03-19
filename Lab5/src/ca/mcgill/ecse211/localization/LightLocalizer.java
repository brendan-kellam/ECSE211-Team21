package ca.mcgill.ecse211.localization;
import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.util.Log;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;

public class LightLocalizer {

	/**
	 * Speed constants for localization
	 */
	private static final int FORWARD_SPEED = 180;
	private static final int ROTATE_SPEED = 120;

	/**
	 * Measured value for the sensor location
	 */
	private static final double SENSOR_LOCATION = 9.3;


	/*
	 * Vehicle components:
	 */
	private double WHEEL_RAD = Vehicle.getConfig().getWheelRadius();
	private EV3LargeRegulatedMotor leftMotor = Vehicle.LEFT_MOTOR;
	private EV3LargeRegulatedMotor rightMotor = Vehicle.RIGHT_MOTOR;

	/*
	 * Odometer and colour sensor.
	 */
	private Odometer odometer;

	/**
	 * RGB Threshold
	 */
	private static final float RGB_DELTA_THRESHOLD = 0.01f;
	/**
	 * Normalized intensity readings from the R, G, B values
	 */
	private float[] curRGB;
	private float[] lastRGB;

	/**
	 * Color sensor provider
	 */
	private SampleProvider csProvider;


	// Is line detected
	private boolean lineDetected = true;

	public LightLocalizer(Odometer odometer)  throws OdometerExceptions {
		this.odometer = odometer;
		this.csProvider = Vehicle.COLOR_SENSOR_BACK.getRGBMode();
		this.curRGB = new float[csProvider.sampleSize()];
	}

	/**
	 * Use the light sensor to localize when AT the origin
	 * @throws OdometerExceptions 
	 */
	public void lightLocalize() throws OdometerExceptions {
		//Initialize the previous RGB values as the current ones.

		this.lastRGB = this.curRGB;

		//Necessary local variables
		double currX, currY, angleX, angleY,angleCorr;
		double[] headingAtLine = new double[4];

		goNearOrigin(); //Blocking method,goes near the origin so that we can find the lines there
		DetectIntersectionLines(headingAtLine); //Detect all 4 lines that meet at the origin

		//Find out our angle using the difference of the lines.

		angleY = headingAtLine[0] - headingAtLine[2]; //Lines 0 and 2 are the vertical lines
		angleX = headingAtLine[1] - headingAtLine[3]; //Lines 1 and 3 are the horizontal lines

		//Use trigonometry to get dx and dy
		currX = -Math.abs(SENSOR_LOCATION * Math.cos(Math.toRadians(angleY / 2)));
		currY = -Math.abs(SENSOR_LOCATION * Math.cos(Math.toRadians(angleX / 2)));


		//Need to correct the angle:
		angleCorr = 270 + (angleY/2) - (headingAtLine[0]); 
		
		odometer.setXYT(currX, currY, (odometer.getXYT()[2] + angleCorr) % 360);
		//If we're not near the origin, get there.
		
		//Just to test the robot, can remove later
		if (!isNearOrigin(currX,currY)) {
			Navigator.travelTo(0, 0, true, true,FORWARD_SPEED);
			odometer.setTheta(odometer.getXYT()[2]-17); // 20 is a magic number
			
			try {
				Thread.sleep(300);
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			
			Vehicle.setMotorSpeeds(30, 30);
			Navigator.turnTo(0); 
			
		}
	}


	/**
	 * 
	 * @param angleAtLines
	 */
	/**
	 * Detect every line that intersects at the origin.
	 * @param angleAtLines
	 */
	private void DetectIntersectionLines(double[] angleAtLines) {
		int currLineDetected = 0;
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		while (currLineDetected < 4) {
			//Rotate ass first in place to find the next line.
			leftMotor.backward();
			rightMotor.forward();
			//When a line is detected, correct the angle and keep track of the values
			if (lineDetected()) {
				angleAtLines[currLineDetected] = odometer.getXYT()[2];
				currLineDetected++;
				Sound.beep();
			}
		}
		Vehicle.setMotorSpeeds(0, 0);
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}


	/**
	 * This method moves the robot towards the origin, which is 45 degrees diagonal from our position
	 */
	public void goNearOrigin() {

		Navigator.turnTo(45);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		//Drive more or less NEAR the origin
		while (!lineDetected()) {
			leftMotor.forward();
			rightMotor.forward();
		}


		leftMotor.stop(true);
		rightMotor.stop();

		// Move backwards so our light sensor can scan the cross at the origin while rotating
		//May want to adjust this value.
		leftMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_LOCATION-8), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_LOCATION-8), false);
	}

	private static boolean isNearOrigin(double x, double y) {
		double error = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
		return error < 2;
	}

	private boolean lineDetected() {
		// Fetch the current RGB intensity sample
		csProvider.fetchSample(curRGB, 0);

		// If a line has already been detected
		if (lineDetected) {

			// Reset the last RGB to the current
			lastRGB = curRGB.clone();
			lineDetected = false;
		}

		// Get RGB components for last reading and currect
		float r1 = lastRGB[0];
		float g1 = lastRGB[1];
		float b1 = lastRGB[2];

		float r2 = curRGB[0];
		float g2 = curRGB[1];
		float b2 = curRGB[2];

		// Compute difference in each component
		float rdiff = r2 - r1;
		float gdiff = g2 - g1;
		float bdiff = b2 - b1;

		// Compute delta between the r, g, b components of the last reading and the current reading
		float delta = (float) Math.sqrt(rdiff*rdiff + gdiff*gdiff + bdiff*bdiff);

		// update last RGB value
		lastRGB = curRGB.clone();

		// If color delta is significant
		if (delta > RGB_DELTA_THRESHOLD) {
			lineDetected = true;
			return true;
		}
		return false;
	}
}
