package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.util.EV3Math;
import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.util.Vehicle;

/**
 * Facade for performing navigation tasks 
 */
public class Navigator {

	// Default vehicle speed
	private static final int DEFAULT_SPEED = 200;
	private static final int DEFAULT_ROTATION_SPEED = 125;

	// Theta threshold
	private static final double THETA_THRESHOLD = 3;
	
	/**
	 * Private constructor: Prevent instances of Navigator existing.
	 * 
	 * Navigator contains <b>static methods</b> only!
	 */
	private Navigator() {
		
	}

	/**
	 * Travel to a specific location in space <br>
	 * 
	 * NOTE: <b> BLOCKING FUNCTION </b>
	 * 
	 * @param x
	 * @param y
	 * @param stop
	 * @param turn
	 * @param speed
	 * @throws OdometerExceptions
	 */
	public static void travelTo(double x, double y, boolean stop, boolean turn, int speed) throws OdometerExceptions {


        Log.log(Log.Sender.Navigator, "TargetX: " + x + " | TargetY: " + y + " | curX: " + Odometer.getX() + " | curY: " + Odometer.getY());

	    
		// If turn is enabled
		if (turn) {
			double minAngle;
			minAngle = getDestAngle(x, y);
			
	        Log.log(Log.Sender.Navigator, "Before Turn Theta: " + Odometer.getTheta());
			
			turnTo(minAngle, speed, false);
			
	        Log.log(Log.Sender.Navigator, "After Turn Theta: " + Odometer.getTheta());
		}
		
		// Compute distance
		double distX = Math.abs(x - Odometer.getX());
		double distY = Math.abs(y - Odometer.getY());
		double dist = Math.hypot(distX, distY);

		// Set motor speed and rotate
		Vehicle.setMotorSpeeds(speed, speed);
		int distance = EV3Math.convertDistance(Vehicle.getConfig().getWheelRadius(), dist);
		if (speed < 0) {
			distance = -distance;
		}

		Vehicle.LEFT_MOTOR.rotate(distance, true);
		Vehicle.RIGHT_MOTOR.rotate(distance, false);


		// Stop at location
		if (stop) {
			Vehicle.setMotorSpeeds(0, 0);
		}
	}

	/**
	 * Travel to a specific location in space <br>
	 * 
	 * NOTE: <b> NONBLOCKING FUNCTION </b>
	 * 
	 * @param x
	 * @param y
	 * @param turn
	 * @throws OdometerExceptions
	 */
	public static void travelToNonBlocking(double x, double y) throws OdometerExceptions {
		//Turn to face it:
		double minAngle;
		minAngle = getDestAngle(x, y);
		turnTo(minAngle, DEFAULT_SPEED, false);

		// Compute distance
		double distX = Math.abs(x - Odometer.getX());
		double distY = Math.abs(y - Odometer.getY());
		double dist = Math.hypot(distX, distY);

		// Set motor speed and rotate
		Vehicle.setMotorSpeeds(DEFAULT_SPEED, DEFAULT_SPEED);
		int distance = EV3Math.convertDistance(Vehicle.getConfig().getWheelRadius(), dist);
		Vehicle.LEFT_MOTOR.rotate(distance, true);
		Vehicle.RIGHT_MOTOR.rotate(distance, true);
	}


	/**
	 * TurnTo function which takes an angle and boolean as arguments. The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 * @param targetAngle
	 * @param immediateReturn
	 */
	public static void turnTo(double theta, int speed, boolean immediateReturn) {

		double currentHeading = 0.0;
		try {
			Odometer odo = Odometer.getOdometer();

			synchronized(odo) {
				currentHeading = odo.getXYT()[2];
			}

		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}

		Log.log(Log.Sender.Navigator, "Current Heading: " + currentHeading);
		Log.log(Log.Sender.Navigator, "Target Heading: " + theta);

		double distance = EV3Math.distance(theta, currentHeading);

		Log.log(Log.Sender.Navigator, "Distance: " + distance);

		if(distance > THETA_THRESHOLD) {

			int cw = findTurnSide(theta, currentHeading);

			Vehicle.LEFT_MOTOR.setSpeed(speed);
			Vehicle.RIGHT_MOTOR.setSpeed(speed);

			double leftRotation = EV3Math.convertAngle(Vehicle.getConfig().getWheelRadius(),Vehicle.getConfig().getTrackWidth(),distance)*-cw;
			double rightRotation = EV3Math.convertAngle(Vehicle.getConfig().getWheelRadius(),Vehicle.getConfig().getTrackWidth(), distance)*cw;

			Vehicle.LEFT_MOTOR.rotate((int) leftRotation, true);
			Vehicle.RIGHT_MOTOR.rotate((int) rightRotation, immediateReturn);
		}
	}


	/**
	 * Given two angles, determine which way the vehicle will turn
	 * 
	 * @param alpha
	 * @param beta
	 * @return -1 -> Left turn | 1 -> Right turn
	 */
	public static int findTurnSide(double alpha, double beta)
	{
		double diff = beta - alpha;
		if(diff < 0)
			diff += 360;
		if(diff > 180)
			return -1; // left turn
			else
				return 1; // right turn
	}

	/**
	 * Get minimum destination angle
	 * 
	 * @param x
	 * @param y
	 * @return
	 * @throws OdometerExceptions 
	 */
	public static double getDestAngle(double x, double y) throws OdometerExceptions {
		double[] position = Odometer.getOdometer().getXYT();
		double curX =     position[0];
		double curY =     position[1];

		double xdiff = x - curX;
		double ydiff = y - curY;

		double atan = Math.atan2(xdiff, ydiff);
		double atanDeg = Math.toDegrees(atan);
		double atanDegBound = EV3Math.boundAngle(atanDeg);
		return atanDegBound;
	}
	
	/**
	 * Travel a specific distance forward
	 * @param distance
	 */
	public static void travelSpecificDistance(double distance) {
	    travelSpecificDistance(distance, DEFAULT_SPEED);
	}

	/**
	 * Travel a specific distance forward, with a desired speed
	 * @param distance
	 */
	public static void travelSpecificDistance(double distance, int speed) {
		// Set motor speed and rotate
		Vehicle.setMotorSpeeds(speed, speed);
		int dist = EV3Math.convertDistance(Vehicle.getConfig().getWheelRadius(), distance);
		Vehicle.LEFT_MOTOR.rotate(dist, true);
		Vehicle.RIGHT_MOTOR.rotate(dist, false);
	}
	/**
	 * Optional turnTo. Uses DEFAULT_SPEED </br>
	 * NOTE: <b> BLOCKING METHOD </b>
	 * 
	 * @param targetAngle
	 * @param stop
	 */
	public static void turnTo(double targetAngle) {
		turnTo(targetAngle, DEFAULT_ROTATION_SPEED, false);
	}
	
	/**
	 * Optional turnTo. Uses DEFAULT_SPEED </br>
	 * NOTE: <b> BLOCKING METHOD </b>
	 * 
	 * @param targetAngle
	 * @param stop
	 */
	public static void turnTo(double targetAngle,int speed) {
		turnTo(targetAngle, speed, false);
	}

	/**
	 * Optional travelTo. Uses DEFAULT_SPEED
	 * 
	 * @param x
	 * @param y
	 * @param stop
	 * @param turn
	 * @throws OdometerExceptions 
	 */
	public static void travelTo(double x, double y, boolean stop, boolean turn) throws OdometerExceptions {
		travelTo(x, y, stop, turn, DEFAULT_SPEED);
	}


}
