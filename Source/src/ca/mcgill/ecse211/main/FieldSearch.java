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
	private static final int maxFilter = 10;
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
		double finalX;
		double finalY;

		// Travel to first waypoint
		Point firstWaypoint = searchArea.popWaypoint();

		//Hold onto these variables incase the can is discovered in the first tile.
		finalX  = firstWaypoint.getX();
		finalY = firstWaypoint.getY();

		Navigator.travelTo((searchArea.getBottomLeft().getX()) * TILE_SIZE, (searchArea.getBottomLeft().getY()) * TILE_SIZE, true, true);
		//Travel to the first waypoint, denoted (Lx+0.5tile, Ly+0.tile)
		try {
			Navigator.travelTo(firstWaypoint.getX(), firstWaypoint.getY(), true, true);
		} catch (OdometerExceptions e1) {
			e1.printStackTrace();
		}

		Navigator.turnTo(0);
		Point waypoint;
		while ((waypoint = searchArea.popWaypoint()) != null) {

			//Runs under the assumption that we're currently at the center of the tile, facing north
			try {
				Thread.sleep(30);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}

			Heading heading = Board.getHeading(Odometer.getTheta());
			double[] targetLocation = new double[2];
			double heldAngle = Odometer.getTheta();

			//Sweep between two angles, depending on direction of bot
			if (heading == Heading.N) {
				targetLocation[0] = 310;
				targetLocation[1] = 60;
			}
			else if (heading == Heading.E || heading == Heading.S){
				targetLocation[0] = 130;//Check from left to right.
				targetLocation[1] = 235;
			} else {
				targetLocation[0] = Odometer.getTheta();
				targetLocation[1] = Odometer.getTheta();
			}

			Thread.sleep(20);
			Navigator.turnTo(targetLocation[0], sweepSpeed, true);

			if (scanForCan(targetLocation[0],cd)) {
				if (usPoller.getDistance() < 3) {
					return true;
				}
			}

			Thread.sleep(20);

			Navigator.turnTo(heldAngle, sweepSpeed, false);
			Thread.sleep(20);

			Navigator.turnTo(targetLocation[1], sweepSpeed, true);

			if (scanForCan(targetLocation[1],cd)) {
				if (usPoller.getDistance() < 5) {
					return true;
				}
			}

			Thread.sleep(50);

			//Travel to it, poll if there's a can. If there is, scan it, back up, dodge it, and continue.

			double destinationX = waypoint.getX();
			double destinationY = waypoint.getY();
			Navigator.turnTo(Navigator.getDestAngle(destinationX, destinationY));
			Navigator.travelSpecificDistance(5);
			dll.travelToLine(200);
			Navigator.travelTo(destinationX, destinationY,true,true);

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
		int maxDistance = 30;

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

	private void goToFinal(double finalX, double finalY) throws OdometerExceptions {

		Navigator.travelTo(searchArea.getTopRight().getX() * TILE_SIZE, searchArea.getTopRight().getY() * TILE_SIZE, true, true);
		//	    
		//		//		Sound.beep();
		//		Navigator.turnTo(0);
		//		//		Sound.beep();
		//		Navigator.travelTo(finalX, (searchArea.getTopRight().getY())  * TILE_SIZE - TILE_SIZE/2, true, false); //works well
		//		//		Sound.beep();
		//		Navigator.turnTo(90);
		//		//		Sound.beep();
		//		Navigator.travelTo((Lab5.URx) * TILE_SIZE - TILE_SIZE/2, (Lab5.URy) * TILE_SIZE - TILE_SIZE/2, true, false); //Never stops going!
		//		//		Sound.beep();
		//		Navigator.turnTo(45);
		//		//		Sound.beep();
		//		Navigator.travelTo((Lab5.URx) * TILE_SIZE, (Lab5.URy-1) * TILE_SIZE, true, false);
	}

	/**
	 * Navigate to the lower left-hand corner of the search area
	 * @throws OdometerExceptions 
	 */
	private void navigateToLL() throws OdometerExceptions {
		// Traveling to bottom left
		Point bottomLeft = searchArea.getBottomLeft();

		double targetX = (bottomLeft.getX() - 1) * TILE_SIZE + TILE_SIZE/1.5;
		double targetY = (bottomLeft.getY() - 1) * TILE_SIZE + TILE_SIZE/1.5;

		// Navigate to lower left
		Navigator.travelTo(targetX, targetY, true, true);
	}

	/*
	 * 
	 */



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