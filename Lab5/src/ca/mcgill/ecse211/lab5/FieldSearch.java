package ca.mcgill.ecse211.lab5;

import java.util.ArrayDeque;
import java.util.Queue;

import ca.mcgill.ecse211.colour.ColourDetection;
import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Board.Heading;
import ca.mcgill.ecse211.util.Log.Sender;
import ca.mcgill.ecse211.util.EV3Math;
import ca.mcgill.ecse211.util.Log;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Point;

public class FieldSearch {


	// Ultrasonic poller
	private UltrasonicPoller usPoller;

	//Ultrasonic sensor
	private static final SensorModes usSensor = Vehicle.US_SENSOR; 
	private static SampleProvider usDistance = usSensor.getMode("Distance"); 
	private static float[] usData = new float[usDistance.sampleSize()];

	// Search area
	private SearchArea searchArea;

	private StartingCorner startingCorner;

	private static final double TILE_SIZE = 30.48;

	private OdometryCorrection correction;
	private final double maxDistance = 26;

	public FieldSearch(SearchArea searchArea, StartingCorner SC, UltrasonicPoller poller, OdometryCorrection correction) {
		this.usPoller = poller;
		this.searchArea = searchArea;
		this.startingCorner = SC;
		this.correction = correction;
	}

	/**
	 * Method shall:: Scan through the grid created from the specified coordinates.
	 * @throws InterruptedException 
	 * @throws OdometerExceptions 
	 * 
	 * 
	 */
	public void startSearch() throws InterruptedException, OdometerExceptions {

		//Keep track of the coordinate we terminate the search at.
		double finalX;
		double finalY;
		
		// Travel to first waypoint
		Point firstWaypoint = searchArea.getNextWaypoint();
		
		//Hold onto these variables incase the can is discovered in the first tile.
		finalX  = firstWaypoint.getX();
		finalY = firstWaypoint.getY();
		
		Navigator.travelTo((Lab5.LLx-1) * TILE_SIZE, (Lab5.LLy-1) * TILE_SIZE, true, true);
		for (int i=0;i<3;i++) {
			Sound.beep();
		}
		//Travel to the first waypoint, denoted (Lx+0.5tile, Ly+0.tile)
		try {
			Navigator.travelTo(firstWaypoint.getX(), firstWaypoint.getY(), true, true);
		} catch (OdometerExceptions e1) {
			e1.printStackTrace();
		}

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e1) {
			e1.printStackTrace();
		}
		Navigator.turnTo(0);

		Point waypoint;
		while ((waypoint = searchArea.getNextWaypoint()) != null) {
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}

			Heading heading = Board.getHeading(Odometer.getTheta());

			double targetLocation;

			if (heading == Heading.E ) {
				targetLocation = 180.0;
			}
			else if (heading == Heading.N || heading == Heading.S){
				targetLocation = 90.0;
			} else {
				targetLocation = Odometer.getTheta();
			}

			//            Navigator.turnTo(targetLocation);
			Thread.sleep(100);
			Navigator.turnTo(targetLocation, 40, true);

			usSensor.fetchSample(usData,0);	
			int currentDistance = (int) (usData[0] * 100.0);

			while (currentDistance > maxDistance && Math.abs(Odometer.getTheta() - targetLocation) > 5)  {

				Log.log(Sender.usSensor, "curDistance: " + currentDistance);
				usSensor.fetchSample(usData,0);	
				currentDistance = (int) (usData[0] * 100.0);
				Thread.sleep(20);
				LCD.drawString(currentDistance + " is the curr distance", 6, 0);
			}

			if (currentDistance < maxDistance ) {
				Sound.buzz();
				if (ColourDetection.checkCanColour(1)) {
					break;
				}
			}

			Thread.sleep(100);

			//correction.enableCorrection();
			Navigator.travelTo(waypoint.getX(), waypoint.getY(), true, true);
			finalX = waypoint.getX();
			finalY = waypoint.getY();
			//correction.disableCorrection();
			Sound.beepSequence();
		}

		goToFinal(finalX,finalY);
		LCD.drawString(":) :) :)", 6, 0);

		for (int i=0;i<3;i++) {
			Sound.beep();
		}
	}


	private void goToFinal(double finalX, double finalY) throws OdometerExceptions {

		Sound.beep();
		Navigator.turnTo(0);
		Sound.beep();
		Navigator.travelTo(finalX, (Lab5.URy-1) * TILE_SIZE - TILE_SIZE/2, true, false); //works well
		Sound.beep();
		Navigator.turnTo(90);
		Sound.beep();
		Navigator.travelTo((Lab5.URx-1) * TILE_SIZE - TILE_SIZE/2, (Lab5.URy-1) * TILE_SIZE - TILE_SIZE/2, true, false); //Never stops going!
		Sound.beep();
		Navigator.turnTo(45);
		Sound.beep();
		Navigator.travelTo((Lab5.URx-1) * TILE_SIZE, (Lab5.URy-1) * TILE_SIZE, true, false);
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
