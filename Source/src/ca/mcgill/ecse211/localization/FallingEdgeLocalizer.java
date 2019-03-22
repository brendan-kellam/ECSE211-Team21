package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

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

	public static final int ROTATE_SPEED = 300;

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
	 * Set the acceleration to a ridiculous amount to speed up the process
	 */
	public void usLocalize() {
		Vehicle.setAcceleration(6000, 6000);
		fallingEdge();
	}


	/**
	 *Localize the robot's position using the falling edge method. In this case, it faces away from the wall for most of the orienting.
	 */
	private void fallingEdge() {

		double angle1, angle2, turnAngle;
		boolean facingWall = true;
		/*
		 * Rotate to face away from a potential wall, then rotate back to obtain angle of right wall
		 */
		Vehicle.setMotorSpeeds(-ROTATE_SPEED, ROTATE_SPEED);// Block until no wall detected
		block(facingWall);
		
		/*
		 * Rotate to obtain the angle at the first wall
		 */
//		Vehicle.setMotorSpeeds(-ROTATE_SPEED, ROTATE_SPEED); // maybe not needed
		
		//Rotate to face the wall
		block(!facingWall); // Block until left wall detected
		angle1 = odometer.getXYT()[2]; //Keep track of the first angle detected.

		/*
		 * Rotate to face the other wall to obtain the angle at the other wall
		 */
		Vehicle.setMotorSpeeds(ROTATE_SPEED, -ROTATE_SPEED);
		block(facingWall);// Get above the tolerance zone so we can rotate to the other wall

//		Vehicle.setMotorSpeeds(ROTATE_SPEED, -ROTATE_SPEED); //maybe not needed
		block(!facingWall);// Block until no wall detected
		while (usPoller.getDistance() > distFallingEdge); 

		angle2 = odometer.getXYT()[2]; //Keep track of the second angle detected.

		turnAngle = getTurnAngle(angle1,angle2);

		//Face to face 45 degrees.
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, turnAngle), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, turnAngle), false);
		
		//Set theta to 0(the x and y coordinates are wrong for now.)
		odometer.setXYT(0.0, 0.0, 45.0);
	}

	/**
	 * Block the method depending on if we want to wait until we're facing the wall or not.
	 * @param facingWall true if we're facing the wall, false otherwise
	 */
	private void block(boolean facingWall) {
		if (facingWall) {
			while (usPoller.getDistance() < distFallingEdge + tolFallingEdge); 
		}
		else {
			while (usPoller.getDistance() > distFallingEdge); 	
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
		return dTheta + odometer.getXYT()[2] - 45; // face 45 degrees
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
