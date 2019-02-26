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
	 * Method shall::
	 * @throws InterruptedException 
	 * 
	 * 
	 */
	public void startSearch() throws InterruptedException {

		/*
        // Navigate to lower left corner
        try {
            navigateToLL();
        } catch (OdometerExceptions e) {
            e.printStackTrace();
        }

        // Turn to face N
        Navigator.turnTo(0.0, true);

        // Traveling to bottom left
        Point bottomLeft = searchArea.getBottomLeft();

        // Localize to field
        LightLocalizer ll = new LightLocalizer(bottomLeft.getX()*TILE_SIZE, bottomLeft.getY()*TILE_SIZE);
        ll.localize();

        // Navigate to center of lower left tile
        try {
            Navigator.travelTo(bottomLeft.getX()*TILE_SIZE + TILE_SIZE/1.5, bottomLeft.getY()*TILE_SIZE + TILE_SIZE/1.5, true, true);
        } catch (OdometerExceptions e) {
            e.printStackTrace();
        }
        Navigator.turnTo(0.0, true);
		 */

		// Travel to first waypoint
		Point firstWaypoint = searchArea.getNextWaypoint();
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
			Navigator.turnTo(targetLocation, 40, true);
			
			usSensor.fetchSample(usData,0);	
			int currentDistance = (int) (usData[0] * 100.0);

			Log.log(Sender.usSensor, "Starts here");

			while (currentDistance > maxDistance && Math.abs(Odometer.getTheta() - targetLocation) > 5)  {

				Log.log(Sender.usSensor, "curDistance: " + currentDistance);
				usSensor.fetchSample(usData,0);	
				currentDistance = (int) (usData[0] * 100.0);
				Thread.sleep(20);
				LCD.drawString(currentDistance + " is the curr distance", 6, 0);
			}

			Log.log(Sender.usSensor, "Ends here: " + currentDistance);
			
			if (currentDistance < maxDistance ) {
				Sound.buzz();
				ColourDetection.checkCanColour(1);
			}

//				Sound.beepSequence();

			try {
				Thread.sleep(4000);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}


			//correction.enableCorrection();

			try {

				Navigator.travelTo(waypoint.getX(), waypoint.getY(), true, true);
			} catch (OdometerExceptions e) {
				e.printStackTrace();
			}
			//correction.disableCorrection();

			Sound.beepSequence();

		}


		Heading curHeading;

		/*
        for (int x = 0; x < (int) searchArea.getWidth(); x++) {
            for (int y = 0; y < (int) searchArea.getHeight(); y++) {

                curHeading = searchArea.getBoard().getHeading(Odometer.getTheta());

                double yoff = TILE_SIZE;
                double turnDir = 45.0;

                // Flip the sign of yoff
                if (curHeading == Heading.S) {
                    yoff = -yoff;
                    turnDir=-45.0;
                    Sound.buzz();
                }

                Navigator.turnTo(turnDir, true);

                // INSERT POLLING HERE
                Sound.twoBeeps();
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e1) {
                    e1.printStackTrace();
                }

                if (usPoller.getDistance() < 15) {
                    Sound.beep();
                }

                try {
                    Navigator.travelTo(Odometer.getX(), Odometer.getY() + yoff, true, true);
                    Sound.beep();
                    Thread.sleep(1000);
                } catch (OdometerExceptions | InterruptedException e) {
                    e.printStackTrace();
                }

            }


            curHeading = searchArea.getBoard().getHeading(Odometer.getTheta());

            // Need to advance vehicle east across the board
            try {
                Navigator.travelTo(Odometer.getX()+TILE_SIZE, Odometer.getY(), true, true);
            } catch (OdometerExceptions e) {
                e.printStackTrace();
            }

            if (x % 2 == 0) {
                Navigator.turnTo(180.0, true);
            } else {
                Navigator.turnTo(0.0, true);
            }

            Sound.beep();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
		 */


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
