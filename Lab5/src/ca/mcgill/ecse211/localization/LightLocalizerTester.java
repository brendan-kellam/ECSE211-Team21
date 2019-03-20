package ca.mcgill.ecse211.localization;
import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.EV3Math;
import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.util.Board.Heading;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;


//TODO: ADD the ability to localize at any coordinate, and still have perfect heading!
//TODO: make it so that after initiali localization, coordinates are set to 1.1


public class LightLocalizerTester {

	/**
	 * Speed constants for localization
	 */
	private static final int FORWARD_SPEED = 180;
	private static final int ROTATE_SPEED = 120;

	/**
	 * Measured value for the sensor location
	 */
	private static final double SENSOR_LOCATION = 10;


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
	private static final float RGB_DELTA_THRESHOLD = 0.015f;
	/**
	 * Normalized intensity readings from the R, G, B values
	 */
	private SensorMode sensorColour;
	private float curIntensity;
	
	/**
	 * Color sensor provider
	 */
	private SampleProvider csProvider;

	// Is line detected
	private boolean lineDetected = true;
	
	// Final heading
	private float offset = 0;

	// Final heading
	private float finalHeading = 0;
	/*
	 * Heading states:
	 */
	private enum Heading {
		NE, // angle between 0 and 90, final heading is 0 
		SE, // angle between 90 and 180, final heading is 90 
		SW, // angle between 180 and 270, final heading is 180 
		NW, // angle between 270 and 360, final heading is 270 
	}

	public LightLocalizerTester(Odometer odometer)  throws OdometerExceptions {
		this.odometer = odometer;
		sensorColour = Vehicle.COLOR_SENSOR_BACK.getRedMode(); //Red sensor is best based on what we learned in lab 2	
	}



	/**
	 * Uses the light sensor to localize around a coordinate.
	 * @param x the x coordinate of the tile
	 * @param y the y coordinate of the tile
	 * @throws OdometerExceptions
	 */
	/**
	 * Uses the light sensor to localize around a coordinate.
	 * @param x the x coordinate of the tile
	 * @param y the y coordinate of the tile
	 * @throws OdometerExceptions
	 */
	public void lightLocalize(double x, double y) throws OdometerExceptions {


		//Necessary local variables
		double currX, currY, arcX, arcY,angleCorr;
		double[] headingAtLine = new double[4];

		boolean origin = false;
		if (x == Board.TILE_SIZE && y == Board.TILE_SIZE) {
			origin = true;
		}

		if (origin) {	
			goNearCoordinate(x,y); //Blocking method,goes near the origin so that we can find the lines there
		}

		else {
			goNearCoordinate(x,y);
			Heading entranceHeading = getHeading();
			
			/*
			 * 
	        NE, // entrance angle between 0 and 90, final heading is ~<90 after correcting
	        SE, // entrance angle between 90 and 180, final heading is ~<180 after correcting
	        SW, // entrance angle between 180 and 270, final heading is ~<270 after correcting 
	        NW, // enrtance angle between 270 and 360, final heading is ~<360 after correcting 
			 */
			
		//It thinks it's facing (90 - tiny amount) upon ending, but it's actually facing ^:
			switch (entranceHeading) {
			case NE: 
			{
				offset = 0; // Should be at the desired offset
			}
			case SE:
			{
				offset = 195; // Add this to reach the desired( offset should be around 180)
			}
			case SW:
			{
				offset = 100; // Add this to reach the desired 180~( offset should be around 100 )
			}
			case NW:
			{
				offset = -80; //Add this to make us reach the desired 360~ ( offset should be around -90 )
			}
			}
		}

		DetectIntersectionLines(headingAtLine); //Detect all 4 lines that meet at the origin

		//Find out our angle using the difference of the lines.

		arcY = headingAtLine[0] - headingAtLine[2]; //Lines 0 and 2 are the vertical lines
		arcX = headingAtLine[1] - headingAtLine[3]; //Lines 1 and 3 are the horizontal lines

		//Use trigonometry to get dx and dy
		currX = -Math.abs(SENSOR_LOCATION * Math.cos(Math.toRadians(arcY / 2)));
		currY = -Math.abs(SENSOR_LOCATION * Math.cos(Math.toRadians(arcX / 2)));


		//Need to correct the angle:
		angleCorr = 270 + (arcY/2) - (headingAtLine[0]); 

		//Maybe there's an error here:
		odometer.setXYT(currX, currY, (odometer.getXYT()[2] + angleCorr) % 360);

		//If we're not near the origin, get there.
		if (!isNearGridIntersection(currX,currY)) {
			Navigator.travelTo(0, 0, true, true,FORWARD_SPEED);
			odometer.setTheta(odometer.getXYT()[2]-15); // 17 is a magic number

			try {
				Thread.sleep(25);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}

			//Set it to 0 if it's the origin
			if (origin) {
				Vehicle.setMotorSpeeds(30, 30);
				Navigator.turnTo(0);
			}
			else {
				odometer.setTheta((odometer.getXYT()[2]+offset) % 360);
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}

		odometer.setXYT(x, y, odometer.getXYT()[2]);

		try {
			Thread.sleep(25);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}


	/**
	 * Refactor this with the go near origin method.
	 * @param x
	 * @param y
	 * @throws OdometerExceptions
	 */
	private void goNearCoordinate(double x,double y) throws OdometerExceptions {

		Navigator.turnTo(Navigator.getDestAngle(x,y));
		//Drive more or less NEAR the coordinate
		Vehicle.setMotorSpeeds(130, 130);
		while (!lineDetected()) ;

		leftMotor.stop(true);
		rightMotor.stop();

		try {
            Thread.sleep(30);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
		// Move backwards so our light sensor can scan the cross at the origin while rotating
		//May want to adjust this value.
		leftMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_LOCATION-7), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_LOCATION-7), false);
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
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
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
		leftMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_LOCATION-7), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_LOCATION-7), false);
	}

	private static boolean isNearGridIntersection(double x, double y) {
		double error = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
		return error < 2;
	}

	private boolean lineDetected() {
		//Get the current intensity. On the blue board, the value is roughyl 350.
		curIntensity = fetchLightSample();
		if (curIntensity < 0.30) {
			return true;
		}
		return false;
	}


	/**
	 * 
	 * @return the intensity of the light sensor
	 */
	private float fetchLightSample() {
		float[] intensity = new float[sensorColour.sampleSize()];
		sensorColour.fetchSample(intensity, 0);
		return intensity[0];
	}
	
	/**
	 * Determine vehicle's heading </br>
	 * 
	 * <b> NOTE: </b> This method will bound theta to: 0 <= theta < 360 <br>
	 * @see EV3Math.boundAngle(theta)
	 * 
	 * @param theta
	 * @return Heading
	 */
	private Heading getHeading() {

		double theta = odometer.getXYT()[2];
		if (theta > 0 && theta <= 90) {
			return Heading.NE;
		} else if (theta > 90 && theta <= 180 ) {
			return Heading.SE;
		} else if (theta > 180 && theta <= 270) {
			return Heading.SW;
		} else if (theta > 270 && theta < 360) {
			return Heading.NW;
		}
		else {
			return Heading.NE;
		}
	}
}