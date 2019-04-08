package ca.mcgill.ecse211.main;

import ca.mcgill.ecse211.localization.DualLightLocalizer;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensor.ColorSensor;
import ca.mcgill.ecse211.sensor.ColourDetection;
import ca.mcgill.ecse211.sensor.UltrasonicPoller;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Vehicle;
import ca.mcgill.ecse211.util.Board.Heading;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Point;

public class FieldSearch {


	// Ultrasonic poller
	private UltrasonicPoller usPoller;

	//Sweep speed:
	private final int sweepSpeed = 100;

	//Ultrasonic sensor
	private static final SensorModes usSensor = Vehicle.US_SENSOR; 
	private static SampleProvider usDistance = usSensor.getMode("Distance"); 
	private static float[] usData = new float[usDistance.sampleSize()];

	//One motor for the light sensor.
	public static final EV3MediumRegulatedMotor lightSensorMotor = Vehicle.FRONT_COLOUR_SENSOR_MOTOR;

	// Add a filter for can detection:
	private static int filter = 0;
	private static final int maxFilter = 6;
	// Search area
	private SearchArea searchArea;

	private StartingCorner startingCorner;
	private static final double TILE_SIZE = 30.48;

	private final double maxDistance = 24;

	DualLightLocalizer dll;

	boolean complete = false;
	public FieldSearch(SearchArea searchArea, UltrasonicPoller usPoller, ColorSensor left,ColorSensor right) {
		this.usPoller = usPoller;
		this.searchArea = searchArea;
		this.dll = new DualLightLocalizer(left,right);

	}

	/**
	 * Method shall:: Scan through the grid created from the specified coordinates.
	 * @throws InterruptedException 
	 * @throws OdometerExceptions 
	 * 
	 * 
	 */
	public boolean startSearch(int desiredCanColour) throws InterruptedException, OdometerExceptions {

		ColourDetection cd = new ColourDetection(desiredCanColour,usPoller);

		//Keep track of the coordinate we terminate the search at.
		Navigator.travelTo((searchArea.getBottomLeft().getX()) * TILE_SIZE, (searchArea.getBottomLeft().getY()) * TILE_SIZE, true, true);

		//MANDATORY: NEEDS TO BEEP 3 TIMES UPON ARRIVING
		for (int i=0;i<3;i++) {
			Sound.beep();
		}

		Point waypoint;
		while ((waypoint = searchArea.popWaypoint()) != null) {

			Thread.sleep(50);
			double[] targetLocation = new double[2];
			double heldAngle = Odometer.getTheta();

			targetLocation[0] = (heldAngle-90) % 360;
			targetLocation[1] = (heldAngle+90) % 360;


			Thread.sleep(70);
			Navigator.turnTo(targetLocation[0], sweepSpeed, true);

			if (scanForCan(targetLocation[0],cd)) {
				return true;
			}

			Thread.sleep(20);

			Navigator.turnTo(heldAngle, sweepSpeed, false);
			Thread.sleep(20);

			Navigator.turnTo(targetLocation[1], sweepSpeed, true);

			if (scanForCan(targetLocation[1],cd)) {
				return true;
			}

			Thread.sleep(50);

			//Travel to it, poll if there's a can. If there is, grab it

			double destinationX = waypoint.getX();
			double destinationY = waypoint.getY();

			Navigator.travelToNonBlocking(destinationX, destinationY);
			//While travelling

			while (!withinError(destinationX,destinationY)) {
				//If we found a can, scan it, then dodge it.
				if (usPoller.getDistance() < 8) {
					Vehicle.setMotorSpeeds(0, 0);
					cd.checkCanColour();
					return true;
				}
			}
			//correction.disableCorrection();
			Sound.beepSequence();
		}
		return false;
	}

	/**
	 * Scan for a can near a given location.
	 * @param targetLocation
	 * @param cd
	 * @return
	 * @throws InterruptedException
	 */
	private boolean scanForCan(double targetLocation, ColourDetection cd) throws InterruptedException {
		//Sound.beep();

		int filter = 0;
		int maxDistance = 24;

		while (filter < maxFilter && Math.abs(Odometer.getTheta() - targetLocation) > 5)  {
			Thread.sleep(20);
			if (	usPoller.getDistance() < maxDistance) {
				filter++;
			}
		}

		//Can detected:
		if (filter >= maxFilter) {
			Vehicle.setMotorSpeeds(0, 0);
			Thread.sleep(50);
			cd.checkCanColour();
			return true;
		}
		//		Sound.twoBeeps();
		return false;
	}

	/* * Check if we're within error of the destination
	 * @param x
	 * @param y
	 * @return
	 * @throws OdometerExceptions
	 */
	private boolean withinError(double x, double y) throws OdometerExceptions {
		double currentX = Odometer.getOdometer().getXYT()[0];
		double currentY = Odometer.getOdometer().getXYT()[1];
		return Math.sqrt(Math.pow((currentX - x), 2) + Math.pow((currentY - y), 2)) < 0.5;
	}


	/**
	 * Defines the starting corner for the vehicle
	 */
	public enum StartingCorner {
		LOWER_LEFT,
		LOWER_RIGHT,
		UPPER_RIGHT,
		UPPER_LEFT
	}


}