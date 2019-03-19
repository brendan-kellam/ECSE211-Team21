package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.colour.ColourDetection;
import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.lab5.FieldSearch.StartingCorner;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.util.Tile;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class SearchGrid {

	// Ultrasonic poller
	private UltrasonicPoller usPoller;

	// desired can colour
	private static final int DESIRED_CAN_COLOUR = 1;
	//Ultrasonic sensor
	private static final SensorModes usSensor = Vehicle.US_SENSOR; 
	private static SampleProvider usDistance = usSensor.getMode("Distance"); 
	private static float[] usData = new float[usDistance.sampleSize()];
	
	//One motor for the light sensor.
	public static final EV3MediumRegulatedMotor lightSensorMotor = Vehicle.FRONT_COLOUR_SENSOR_MOTOR;

	// Search area
	private Grid grid;

	private static final double TILE_SIZE = 30.48;

	private OdometryCorrection correction;
	private final double maxDistance = 26;

	public SearchGrid(Grid grid, UltrasonicPoller poller) {
		this.usPoller = poller;
		this.grid = grid;
	}
	
	
	public void search(Grid grid) throws OdometerExceptions {
		//Travel to the lower left corner first tile.
		Tile firstTile = grid.getFirst();
		Navigator.travelTo(firstTile.getLowerLeft().getX(), firstTile.getLowerLeft().getY(), true, true);
		
		//Sweep the first tile.
		scanForCan
		
		/*
		 * Now, we want to search them.
		 */
	}
	
	private boolean scanForCan(double targetLocation) throws InterruptedException {
		//Sound.beep();
		usSensor.fetchSample(usData,0);	
		int currentDistance = (int) (usData[0] * 100.0);

		while (true) {
			if (currentDistance) {
				
			}
			usSensor.fetchSample(usData,0);	
			Thread.sleep(20);
			currentDistance = (int) (usData[0] * 100.0);
		}
		while (currentDistance > maxDistance && Math.abs(Odometer.getTheta() - targetLocation) > 5)  {

			usSensor.fetchSample(usData,0);	
			currentDistance = (int) (usData[0] * 100.0);
			Thread.sleep(20);
			LCD.drawString(currentDistance + " is the curr distance", 6, 0);
		}

		//Can detected:
		if (currentDistance < maxDistance ) {
			Vehicle.setMotorSpeeds(0, 0);
			Thread.sleep(50);
			if (ColourDetection.checkCanColour(DESIRED_CAN_COLOUR)) {
				Sound.beep();
				return true;
			}
		}
		Sound.twoBeeps();
		return false;
	}
}
