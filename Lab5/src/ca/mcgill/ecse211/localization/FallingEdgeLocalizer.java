package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class FallingEdgeLocalizer {

	
	private EV3LargeRegulatedMotor leftMotor = Vehicle.LEFT_MOTOR;
	private EV3LargeRegulatedMotor rightMotor = Vehicle.RIGHT_MOTOR;
	private double WHEEL_RAD = Vehicle.getConfig().getWheelRadius();
	private double TRACK = Vehicle.getConfig().getTrackWidth();
	public static final int ROTATE_SPEED = 150;

    private UltrasonicPoller usPoller; // Ultrasonic poller
	private Odometer odometer; // Odometer

	private static final int distFallingEdge = 30;
	private static final int tolFallingEdge = 2;
	
	public FallingEdgeLocalizer(Odometer odometer,UltrasonicPoller usPoller) {
		this.odometer = odometer;
		this.usPoller = usPoller;
	}

	/**
	 * Check which ultrasonic localization we want to use
	 */
	public void usLocalize() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		fallingEdge();
	}

	
	/**
	 *Localize the robot's position using the falling edge method. In this case, it faces away from the wall for most of the orienting.
	 */
	private void fallingEdge() {

		double angle1, angle2, turnAngle;

		//Rotate to face away from the wall:
		while (usPoller.getDistance() < distFallingEdge + tolFallingEdge) {
			leftMotor.backward();
			rightMotor.forward();
		}
		
		//Rotate to face the wall
		while (usPoller.getDistance() > distFallingEdge) {
			leftMotor.backward();
			rightMotor.forward();
		}
		
		angle1 = odometer.getXYT()[2]; //Keep track of the first angle detected.

		//Rotate to face away from the wall
		while (usPoller.getDistance() < distFallingEdge + tolFallingEdge) {
			leftMotor.forward();
			rightMotor.backward();
		}

		//Rotate to face the other wall
		while (usPoller.getDistance() > distFallingEdge) {
			leftMotor.forward();
			rightMotor.backward();
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
