package ca.mcgill.ecse211.localization;
import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class mattLocalize {

	private static final int FORWARD_SPEED = 180;
	private static final int ROTATE_SPEED = 130;
	private static final double SENSOR_LOCATION = 8.0;

	private double WHEEL_RAD; //initially 2.2
	private double TRACK; //initially 17

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private Odometer odometer;
	private EV3ColorSensor lightSensor;
	private SensorMode sensorColour;
	private float curIntensity;
	double[] angleAtLines;


	public mattLocalize(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3ColorSensor lightSensor,double wheel_rad)  throws OdometerExceptions {

		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.lightSensor = lightSensor;
		this.WHEEL_RAD = wheel_rad;

		sensorColour = lightSensor.getRedMode(); //Red sensor is best based on what we learned in lab 2
		angleAtLines = new double[4];//Hold the angles that we find all 4 of our lines at. 
	}

	/**
	 * Use the light sensor to localize when AT the origin
	 */
	public void lightLocalize() {

		int currLineDetected = 0;//Count how many lines we've detected thus far.
		double currX;
		double currY;
		double angleX;
		double angleY;

		curIntensity = fetchLightSample();
		int value = (int) (curIntensity*1000);
		LCD.drawInt((int) value, 0, 5); // Just to see what values im getting. --M

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		//Go near the origin, so that we can find the lines there
		goNearOrigin();

		//Detect all 4 lines that meet at the origin
		while (currLineDetected < 4) {
			//Rotate ass first in place to find the next line.
			leftMotor.backward();
			rightMotor.forward();
			curIntensity = fetchLightSample(); // Get the intensity of the floor
			value = (int) (curIntensity*1000);
			LCD.drawInt((int) value, 0, 5); // Just to see what values I'm getting.

			//Black line at 0.3 intensity.
			//Blue at 0.35 intensity.
			if (curIntensity < 0.3) {
				//if (Math.abs(curIntensity - 0.35) > 0.03){ 
				angleAtLines[currLineDetected] = odometer.getXYT()[2];
				currLineDetected++;
				Sound.beep();
			}
		}

		leftMotor.stop(true);
		rightMotor.stop();

		//Find out our angle using the difference of the lines.
		angleY = angleAtLines[2] - angleAtLines[0]; //Lines 0 and 2 are the vertical lines
		angleX = angleAtLines[3] - angleAtLines[1]; //Lines 1 and 3 are the horizontal lines

		//Use trigonometry to get dx and dy
		currX = -Math.abs(SENSOR_LOCATION * Math.cos(Math.toRadians(angleY / 2)));
		currY = -Math.abs(SENSOR_LOCATION * Math.cos(Math.toRadians(angleX / 2)));

		//When this wasn't here shit didn't work soooo
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		//Travel to the origin once and for all!
		odometer.setXYT(currX, currY, odometer.getXYT()[2]);
		if (!nearOrigin(currX,currY)) {
			try {
				Navigator.travelTo(0, 0, true, true);
			} catch (OdometerExceptions e) {
				e.printStackTrace();
			}
		}

		//Turn to face 0 degrees(north, basically).	
		double currAngle = odometer.getXYT()[2];
		if (currAngle <= 355 && currAngle >= 5.0) {
			//			Navigator.turnTo(0, ROTATE_SPEED, true);
			Navigator.turnTo(0);
			//			while (curIntensity > 0.3) {
			//				curIntensity = fetchLightSample();
			//			}
			Vehicle.setMotorSpeeds(0, 0);
			Sound.beep();
		}
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
	 * 
	 * @return the intensity of the light sensor
	 */
	private float fetchLightSample() {
		float[] intensity = new float[sensorColour.sampleSize()];
		sensorColour.fetchSample(intensity, 0);
		return intensity[0];
	}

	/**
	 * This method moves the robot towards the origin
	 */
	public void goNearOrigin() {


		Navigator.turnTo(45);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		//Get the current intensity. On the blue board, the value is roughyl 350.
		curIntensity = fetchLightSample();

		//Drive more or less NEAR the origin
		//Lines are at 0.3 intensity
		while (curIntensity > 0.3) {
			curIntensity = fetchLightSample();
			leftMotor.forward();
			rightMotor.forward();
		}


		leftMotor.stop(true);
		rightMotor.stop();
		// Move backwards so our light sensor can scan the cross at the origin while rotating

		leftMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_LOCATION-6), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_LOCATION-6), false);
	}

	private static boolean nearOrigin(double x, double y) {
		double error = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
		return error < 2;
	}

}
