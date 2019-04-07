package ca.mcgill.ecse211.main;

import java.io.FileNotFoundException;

import ca.mcgill.ecse211.Test;
import ca.mcgill.ecse211.localization.DualLightLocalizer;
import ca.mcgill.ecse211.localization.FallingEdgeLocalizer;
import ca.mcgill.ecse211.localization.LightLocalizerTester;
import ca.mcgill.ecse211.localization.DualLightLocalizer.Config;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensor.UltrasonicPoller;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Board.Heading;
import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.util.Log.Sender;
import ca.mcgill.ecse211.util.Tile;
import ca.mcgill.ecse211.util.Vehicle;
import ca.mcgill.ecse211.util.WifiController;
import lejos.hardware.Button;
import lejos.hardware.Sound;

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
		Vehicle.newConfig(new Vehicle.Configuration(2.1, 17.73));
		Vehicle.LEFT_MOTOR.setAcceleration(3000);
		Vehicle.RIGHT_MOTOR.setAcceleration(3000);
        
		// Starting corner
	

		// Create odometer
		Odometer odometer = Odometer.getOdometer();

		// Create ultrasonic poller
		UltrasonicPoller usPoller = new UltrasonicPoller(Vehicle.US_SENSOR);

		FieldSearch.StartingCorner SC = FieldSearch.StartingCorner.LOWER_LEFT;

		// Initialize logging
		Log.setLogging(true, true, true, true, true, true);

		// Set logging to write to file
		try {
			Log.setLogWriter("FinalProj" + ".log");
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		}

		PollerSystem pollerSystem = PollerSystem.getInstance();
		pollerSystem.addPoller(usPoller);
		pollerSystem.addPoller(odometer);
		pollerSystem.start();
		
		FallingEdgeLocalizer ul = new FallingEdgeLocalizer(odometer,usPoller);
		DualLightLocalizer dll = new DualLightLocalizer(Vehicle.LEFT_CS, Vehicle.RIGHT_CS);
		
		
		// ----- Configuration ------
        WifiController.fetchGameplayData();
        Log.log(Sender.usSensor, "Tunnel LL: " + CompetitionConfig.tunnelEntranceToSearchArea.toString());

		
        // ----- Localize to grid ------
        localize(ul, dll);
		
		Log.log(Sender.odometer, "SET XY - X: " + Odometer.getX() + " | Y: " + Odometer.getY());
		
		int desiredCanColour = 1;
		
	    
		// ------ TRAVEL TO TUNNEL -------
		travelToTunnel(dll);	  
        dll.localizeToSquare(Heading.N, Heading.E, Config.BACKWARD);
		travelThroughTunnel();
	    dll.localizeToSquare(Heading.N, Heading.E, Config.FORWARD);
		
        Navigator.travelTo(CompetitionConfig.searchAreaLL.getLowerLeft().getX(), CompetitionConfig.searchAreaLL.getLowerLeft().getY(), true, true);
        
        //Beep 5 times
        for (int i=0;i<5;i++) Sound.beep();
        
        int LLx = (int) (CompetitionConfig.searchAreaLL.getLowerLeft().getX() / Board.TILE_SIZE);
        int LLy = (int) (CompetitionConfig.searchAreaLL.getLowerLeft().getY() / Board.TILE_SIZE);
        int URx = (int) (CompetitionConfig.searchAreaUR.getUpperRight().getX() / Board.TILE_SIZE);
        int URy = (int) (CompetitionConfig.searchAreaUR.getUpperRight().getY() / Board.TILE_SIZE);

        
        // Starting corner [0, 3]
        SearchArea searchArea = new SearchArea(LLx, LLy, URx, URy);
        // Search the field
        FieldSearch fieldSearch = new FieldSearch(searchArea, usPoller,Vehicle.LEFT_CS,Vehicle.RIGHT_CS);
        
        fieldSearch.startSearch(desiredCanColour);
        
        //Beep 5 times
        for (int i=0;i<5;i++) Sound.beep();
		
		pollerSystem.stop();

		// Wait
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
	
	/**
	 * Localize to play board
	 * 
	 * @param ul - falling edge localizer
	 * @param dl - dual light localizer
	 * @throws OdometerExceptions 
	 * @throws InterruptedException 
	 */
	private static void localize(FallingEdgeLocalizer ul, DualLightLocalizer dll) throws OdometerExceptions, InterruptedException {
	    ul.usLocalize();
        dll.localizeToIntersection(Heading.N);
        
        Odometer.getOdometer().setX(Board.TILE_SIZE);
        Odometer.getOdometer().setY(Board.TILE_SIZE);
        
        // Compliance beeps
        for (int i = 0; i < 3; i++) {
            Sound.beep();
        }
        
        dll.travelToLine(100);
	}
	
	/**
	 * Routine to travel to the tunnel
	 * 
	 * @param dll
	 * @throws OdometerExceptions
	 */
	private static void travelToTunnel(DualLightLocalizer dll) throws OdometerExceptions {
	    
	    // Get tunnel lower left and right
	    Tile tunnelLR = CompetitionConfig.tunnelEntranceToSearchArea;
        Tile tunnelUR = CompetitionConfig.tunnelEntranceToStartArea;
        
        // Debug information
        Log.log(Sender.Navigator, "tunnelLR: " + tunnelLR.toString());
        Log.log(Sender.Navigator, "tunnelUR: " + tunnelUR.toString());
        
        
        double targetX = tunnelLR.getLowerLeft().getX();
        double targetY = tunnelLR.getLowerLeft().getY();
        
        Navigator.travelTo(Odometer.getX(), targetY, true, true);
        dll.travelToLine(100);
        
        Navigator.travelSpecificDistance(5);
        
        Navigator.turnTo(Navigator.getDestAngle(targetX, tunnelLR.getCenter().getY()));
        dll.travelToLine(100);
        
        
        Navigator.travelTo(targetX, tunnelLR.getCenter().getY(), true, true);
        dll.travelToLine(100); 
        Navigator.travelSpecificDistance(5);
        
	}

	/**
	 * 
	 * @throws OdometerExceptions
	 */
	private static void travelThroughTunnel() throws OdometerExceptions {

        LightLocalizerTester uc = new LightLocalizerTester(Odometer.getOdometer());
        
        //Cheat the beginning.
        Vehicle.setAcceleration(3000, 3000);
        //Cheat the beginning.
        Navigator.travelSpecificDistance(Board.TILE_SIZE*2,(int) Vehicle.RIGHT_MOTOR.getMaxSpeed()/2);

        Vehicle.setMotorSpeeds(550, 550);
        while (uc.bridgeDetected());

        Vehicle.setMotorSpeeds(0, 0);

    }
	
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