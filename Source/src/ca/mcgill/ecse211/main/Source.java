package ca.mcgill.ecse211.main;

import java.awt.geom.Point2D;
import java.io.FileNotFoundException;

import ca.mcgill.ecse211.Test;
import ca.mcgill.ecse211.colour.ColourDetection;
import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.LightLocalizerTester;
import ca.mcgill.ecse211.localization.FallingEdgeLocalizer;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Display;
import ca.mcgill.ecse211.util.EV3Math;
import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.util.WifiController;
import ca.mcgill.ecse211.util.Log.Sender;
import ca.mcgill.ecse211.util.Tile;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Main entry point class for Lab4
 */

public final class Source {

	// Time to wait after display initialization (to allow graphics to apear on EV3's screen)
	private static final short DISPLAY_INIT_SLEEP_TIME = 2000;
	// Lower left and upper right corner definitions [0,8]

	/**
	 * Represents a given MenuOption
	 */
	private enum MenuOption {
		TEST_COLOURS,
		NAVIGATE_MAP,
		INVALID
	}

	public static final boolean TESTING = true;

	/**
	 * Main entry of program
	 * @param args
	 * @throws OdometerExceptions 
	 * @throws InterruptedException 
	 */
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {


		if (TESTING) {
			Test tester = new Test();
			tester.test();
			return;
		}
		
		// ----- Configuration ------

		// Create new vehicle configuration
		Vehicle.newConfig(new Vehicle.Configuration(2.1, 17.7));
		Vehicle.LEFT_MOTOR.setAcceleration(200);
		Vehicle.RIGHT_MOTOR.setAcceleration(200);
        
		// Starting corner
	

		// Create odometer
		Odometer odometer = Odometer.getOdometer();

		// Create odometery correction | disable correction
		OdometryCorrection odoCorrection = new OdometryCorrection(Vehicle.COLOR_SENSOR_LEFT);
		odoCorrection.disableCorrection();

		// Create ultrasonic poller
		UltrasonicPoller usPoller = new UltrasonicPoller(Vehicle.US_SENSOR);

		// Create new display object
//		Display odometryDisplay = new Display(Vehicle.LCD_DISPLAY);

		FieldSearch.StartingCorner SC = FieldSearch.StartingCorner.LOWER_LEFT;

		

		// Initialize logging
		Log.setLogging(true, false, false, false, false);

		// Set logging to write to file
		try {
			Log.setLogWriter("Lab5" + ".log");
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		}

		// Start odometer correction thread
//		Thread odoCorrectionThread = new Thread(odoCorrection);
//		odoCorrectionThread.start();

		
		PollerSystem pollerSystem = PollerSystem.getInstance();
		pollerSystem.addPoller(usPoller);
		pollerSystem.addPoller(odometer);
		pollerSystem.start();
		
		FallingEdgeLocalizer ul = new FallingEdgeLocalizer(odometer,usPoller);
		LightLocalizerTester uc = new LightLocalizerTester(odometer);

		
		// ----- Configuration ------
        WifiController.fetchGameplayData();
        Log.log(Sender.usSensor, "Tunnel LL: " + Board.Config.tunnelLL.toString());


//			Thread odoDisplayThread = new Thread(odometryDisplay);
//			odoDisplayThread.start(); 

		ul.usLocalize();
		uc.lightLocalize(Board.TILE_SIZE,Board.TILE_SIZE);
		
		Tile tunnelLR = Board.Config.tunnelLL;
		Tile tunnelUR = Board.Config.tunnelUR;
		
		int desiredCanColour = 1;
		
		Log.log(Sender.Navigator, "tunnelLR: " + tunnelLR.toString());
		Log.log(Sender.Navigator, "tunnelUR: " + tunnelUR.toString());
		
		Navigator.travelTo(tunnelLR.getCenter().getX(), tunnelLR.getCenter().getY(), true, true, 200);
		
		
		Thread.sleep(500);
		
		
		Log.log(Log.Sender.avoidance, "HELLOO: X: " + Odometer.getX() + " | Y: " + Odometer.getY() + " | T: " + Odometer.getTheta());
        
		int ya = (int) (Board.Config.tunnelLL.getLowerLeft().getY() / Board.TILE_SIZE);
		
		
		Navigator.travelTo(tunnelUR.getCenter().getX()+5, tunnelUR.getCenter().getY()+5-ya, true, true, 200);	

        uc.lightLocalize(tunnelUR.getUpperRight().getX(), tunnelUR.getUpperRight().getY());

        Navigator.travelTo(Board.Config.searchAreaLL.getLowerLeft().getX(), Board.Config.searchAreaLL.getLowerLeft().getY(), true, true);
        
        //Beep 5 times
        for (int i=0;i<5;i++) Sound.beep();
        
        int LLx = (int) (Board.Config.searchAreaLL.getLowerLeft().getX() / Board.TILE_SIZE);
        int LLy = (int) (Board.Config.searchAreaLL.getLowerLeft().getY() / Board.TILE_SIZE);
        int URx = (int) (Board.Config.searchAreaUR.getUpperRight().getX() / Board.TILE_SIZE);
        int URy = (int) (Board.Config.searchAreaUR.getUpperRight().getY() / Board.TILE_SIZE);

        
        // Starting corner [0, 3]
        SearchArea searchArea = new SearchArea(LLx, LLy, URx, URy, SC);
        // Search the field
        FieldSearch fieldSearch = new FieldSearch(searchArea, SC, usPoller, odoCorrection);
        
        fieldSearch.startSearch(desiredCanColour);
        //Beep 5 times
        for (int i=0;i<5;i++) Sound.beep();
		
		pollerSystem.stop();

		// Wait
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

	//	/**
	//	 * Executes ultrasonic localization
	//	 * 
	//	 * @param ul
	//	 * @param option
	//	 */
	//	private static void executeUSLocalization(UltrasonicPoller poller, MenuOption option) {
	//		UltrasonicLocalizer ul;
	//		if (option == MenuOption.FALLING_EDGE) {
	//			ul = new FallingEdgeLocalizer(poller);
	//		} else {
	//			ul = new RisingEdgeLocalizer(poller);
	//		}
	//
	//		// Localize
	//		ul.localize();
	//	}

	/**
	 * Gets the User's menu choice of either RISING or FALLING edge
	 * 
	 * @return MenuOption
	 */
	public static MenuOption getUserChoice() {
		Vehicle.LCD_DISPLAY.drawString("Test Colours: UP", 0, 0);
		Vehicle.LCD_DISPLAY.drawString("Navigate map: DOWN", 0, 1);

		int choice = Button.waitForAnyPress();

		if (choice == Button.ID_UP) {
			return MenuOption.TEST_COLOURS;
		}
		else if (choice == Button.ID_DOWN) {
			return MenuOption.NAVIGATE_MAP;
		}

		return MenuOption.INVALID;
	}

}
