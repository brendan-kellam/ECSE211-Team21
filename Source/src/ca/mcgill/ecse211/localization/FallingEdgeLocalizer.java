package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.sensor.UltrasonicPoller;
import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.util.Vehicle;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class FallingEdgeLocalizer {


	/*
	 * Motors
	 */
	private EV3LargeRegulatedMotor leftMotor = Vehicle.LEFT_MOTOR;
	private EV3LargeRegulatedMotor rightMotor = Vehicle.RIGHT_MOTOR;

	/*
	 * 
	 */
	private double WHEEL_RAD = Vehicle.getConfig().getWheelRadius();
	private double TRACK = Vehicle.getConfig().getTrackWidth();

	public static final int ROTATE_SPEED = 400;

	private UltrasonicPoller usPoller; // Ultrasonic poller
	private Odometer odometer; // Odometer

	private static final int distFallingEdge = 30;
	private static final int tolFallingEdge = 10;

	public FallingEdgeLocalizer(Odometer odometer,UltrasonicPoller usPoller) {
		this.odometer = odometer;
		this.usPoller = usPoller;
	}

	/**
	 * Check which ultrasonic localization we want to use
	 * Set the acceleration to a ridiculous amount to speed up the process
	 * @throws InterruptedException 
	 */
	public void usLocalize() throws InterruptedException {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		Vehicle.setAcceleration(6000, 6000);
		fallingEdge();
	}


	/**
	 *Localize the robot's position using the falling edge method. In this case, it faces away from the wall for most of the orienting.
	 * @throws InterruptedException 
	 */
	private void fallingEdge() throws InterruptedException {

		double angle1;
		double angle2;
		double turnAngle;

		//Rotate to face away from the wall:
		while (usPoller.getDistance() < distFallingEdge + tolFallingEdge) {
			leftMotor.backward();
			rightMotor.forward();
			Thread.sleep(25);
		}
		
			Thread.sleep(50);
		//Rotate to face the wall
		while (usPoller.getDistance() > distFallingEdge) {
			leftMotor.backward();
			rightMotor.forward();

			Thread.sleep(25);
		}
			Thread.sleep(50);
		angle1 = odometer.getXYT()[2]; //Keep track of the first angle detected.

		//Rotate to face away from the wall
		while (usPoller.getDistance() < distFallingEdge + tolFallingEdge) {
			leftMotor.forward();
			rightMotor.backward();
			Thread.sleep(25);
		}
		Thread.sleep(50);
		//Rotate to face the other wall
		while (usPoller.getDistance() > distFallingEdge) {
			leftMotor.forward();
			rightMotor.backward();

			Thread.sleep(25);
		}
		

		angle2 = odometer.getXYT()[2]; //Keep track of the second angle detected.

		double dTheta = 0;
		//Compute the angle:
		//Case 1: The first angle is smaller than the second=>The second angle comes after the first in a clockwise manner with 0 being 0 degrees
		if (angle1 < angle2) {
			dTheta = 45 - (angle1 + angle2) / 2;

		} 

		//Case 2: The first angle is greater than the second=>The second angle comes BEFORE the first in a clockwise manner with 0 being 0 degrees.
		else if (angle1 > angle2) {
			dTheta = 225 - (angle1 + angle2) / 2;
		}
//
		dTheta-=2;
		turnAngle = dTheta + odometer.getXYT()[2];
		//Face 0 degrees.
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, turnAngle), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, turnAngle), false);
		//Set theta to 0(the x and y coordinates are wrong for now.)
		odometer.setXYT(0.0, 0.0, 0.0);

	}

	/**
	 * Block the method depending on if we want to wait until we're facing the wall or not.
	 * @param facingWall true if we're facing the wall, false otherwise
	 */
	private void block(boolean facingWall) {
		if (facingWall) {
			while (usPoller.getDistance() < distFallingEdge + tolFallingEdge) {
				Log.log(Log.Sender.USLocalization, usPoller.getDistance() + " Is the distance when facing the wall");
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}; 
		}
		else {
			while (usPoller.getDistance() > distFallingEdge) {
				Log.log(Log.Sender.USLocalization, usPoller.getDistance() + " Is the distance when NOT facing the wall");
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}

	/**
	 * Compute the angle the robot must turn to face 45 degrees
	 * @param angle1 the angle obtained when the first wall was detected
	 * @param angle2 the second angle obtained when the second wall was detected
	 * @return the angle the robot must turn to face 45 degrees
	 */
	private double getTurnAngle(double angle1, double angle2) {
		double dTheta;
		//Compute the angle:
		//Case 1: The first angle is smaller than the second=>The second angle comes after the first in a clockwise manner with 0 being 0 degrees
		if (angle1 < angle2) {
			dTheta = 45 - (angle1 + angle2) / 2;
		} 

		//Case 2: The first angle is greater than the second=>The second angle comes BEFORE the first in a clockwise manner with 0 being 0 degrees.
		else {
			dTheta = 225 - (angle1 + angle2) / 2;
		}

		dTheta-=2; // Magic number
		return dTheta + odometer.getXYT()[2]; // face 0 degrees
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius of the tire
	 * @param distance we want to travel
	 * @return the int value we must rotate the wheels to travel a specific distance
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Converts a desired angle into a required distance to rotate for each wheel.
	 * @param radius wheel radius in cm
	 * @param track the width of the car
	 * @param angle desired angle in degrees
	 * @return distance that the car must rotate to arrive at desired angle
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}