package ca.mcgill.ecse211.lab5;

import java.io.FileNotFoundException;
import ca.mcgill.ecse211.colour.ColourDetection;
import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.localization.FallingEdgeLocalizer;
import ca.mcgill.ecse211.localization.RisingEdgeLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.util.Display;
import ca.mcgill.ecse211.util.Log;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 * Main entry point class for Lab4
 */
public final class Lab5 {

	// Time to wait after display initialization (to allow graphics to apear on EV3's screen)
	private static final short DISPLAY_INIT_SLEEP_TIME = 2000;

	/**
	 * Represents a given MenuOption
	 */
	private enum MenuOption {
		TEST_COLOURS,
		NAVIGATE_MAP,
		INVALID
	}


	/**
	 * Main entry of program
	 * @param args
	 * @throws OdometerExceptions 
	 * @throws InterruptedException 
	 */
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {


		// ----- Configuration ------


		// Create new vehicle configuration
		Vehicle.newConfig(new Vehicle.Configuration(2.1, 14.15));
		Vehicle.LEFT_MOTOR.setAcceleration(100);
		Vehicle.RIGHT_MOTOR.setAcceleration(100);


		// Lower left and upper right corner definitions [0,8]
		int LLx = 2, LLy = 2;
		int URx = 6, URy = 6;

		// Target can [1, 4]
		int TR = 3;

		// Starting corner
		FieldSearch.StartingCorner SC = FieldSearch.StartingCorner.LOWER_LEFT;

		// Starting corner [0, 3]
		SearchArea searchArea = new SearchArea(LLx, LLy, URx, URy, SC);

		// Create odometer
		Odometer odometer = Odometer.getOdometer();

		// Create odometery correction | disable correction
		OdometryCorrection odoCorrection = new OdometryCorrection(Vehicle.LEFT_COLOR_SENSOR, Vehicle.RIGHT_COLOR_SENSOR);
		odoCorrection.disableCorrection();

		// Create ultrasonic poller
		UltrasonicPoller usPoller = new UltrasonicPoller(Vehicle.US_SENSOR);

		// Create new display object
		Display odometryDisplay = new Display(Vehicle.LCD_DISPLAY);

		// Search the field
		FieldSearch fieldSearch = new FieldSearch(searchArea, SC, usPoller, odoCorrection);

		// Initialize logging
		Log.setLogging(true, true, true, true, true);

		// Set logging to write to file
		try {
			Log.setLogWriter("Lab5" + ".log");
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		}

		// Get user option
		MenuOption option;
		while ((option = getUserChoice()) == MenuOption.INVALID);

		// Start Odometer Thread
		Thread odoThread = new Thread(odometer);
		odoThread.start();

//		// Start odometer correction thread
//		Thread odoCorrectionThread = new Thread(odoCorrection);
//		odoCorrectionThread.start();

		// Start ultrasonic poller thread 
		Thread usThread = new Thread(usPoller);
		usThread.start();

        // Sleep to allow Display to initialize
        try {
            Thread.sleep(DISPLAY_INIT_SLEEP_TIME);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        
        /*
        LightLocalizer uc = new LightLocalizer(0.0, 0.0);

        executeUSLocalization(usPoller, option);
        uc.localize();
		 */

		Sound.twoBeeps();
		if (option == MenuOption.TEST_COLOURS){
			ColourDetection.forDemoOneToAnalyzeCansOneAfterEachOther();	
		}	
		
		else {
			// Start Display thread.
			//This is here so the display thread doesn't run during colour testing
			
			// Sleep to allow Display to initialize
			try {
				Thread.sleep(DISPLAY_INIT_SLEEP_TIME);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start(); 
			fieldSearch.startSearch();
		}
		
		Sound.beep();

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
