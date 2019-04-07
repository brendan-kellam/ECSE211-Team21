package ca.mcgill.ecse211.main;

import java.awt.geom.Point2D;
import java.io.FileNotFoundException;

import ca.mcgill.ecse211.Test;
import ca.mcgill.ecse211.claw.Claw;
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

	/**
	 * Represents a given MenuOption
	 */
	private enum MenuOption {
		TEST_COLOURS,
		NAVIGATE_MAP,
		INVALID
	}

	public static final boolean TESTING = false;
	
	

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
        	

		// Create odometer
		Odometer odometer = Odometer.getOdometer();

		// Create ultrasonic poller
		UltrasonicPoller usPoller = new UltrasonicPoller(Vehicle.US_SENSOR);
		
		// Initialize logging
		Log.setLogging(true, true, true, true, true, true);

		// Set logging to write to file
		try {
			Log.setLogWriter("FinalProj" + ".log");
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		}

		// Create claw
		Claw claw = new Claw(usPoller);
		
		PollerSystem pollerSystem = PollerSystem.getInstance();
		pollerSystem.addPoller(usPoller);
		pollerSystem.addPoller(odometer);
		pollerSystem.start();
		
		FallingEdgeLocalizer ul = new FallingEdgeLocalizer(odometer,usPoller);
		DualLightLocalizer dll = new DualLightLocalizer(Vehicle.LEFT_CS, Vehicle.RIGHT_CS);
		
		
		// ----- Configuration ------
        WifiController.fetchGameplayData();
        //Log.log(Sender.usSensor, "Tunnel LL: " + CompetitionConfig.tunnelEntranceToSearchArea.toString());
        Log.log(Sender.board, CompetitionConfig.tostr());
		
        // ----- Localize to grid ------
        localize(ul, dll);
        claw.stow();
		
		Log.log(Sender.odometer, "SET XY - X: " + Odometer.getX() + " | Y: " + Odometer.getY());
		
		int desiredCanColour = 1;
		
	    
		// ------ TRAVEL TO TUNNEL -------
		travelToTunnel(dll);	  
		
		Heading toSearchArea = CompetitionConfig.toSearchAreaHeading;
		
		// Traveling to search area
		dll.localizeToTile(toSearchArea, Board.getOrthogonalHeading(toSearchArea), Board.getParallelHeading(toSearchArea));
		travelThroughTunnel(CompetitionConfig.tunnelEntranceToStartArea);
	    dll.localizeToTile(toSearchArea, Board.getOrthogonalHeading(toSearchArea), toSearchArea);
	    
	    claw.release();
	    
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
        
        if (fieldSearch.startSearch(desiredCanColour)) {
            claw.grab();
        }
        
        //Beep 5 times
        for (int i=0;i<5;i++) Sound.beep();
		
        
        // NAVIGATE BACK TO START AREA
        
        Tile tunnelEntrance = CompetitionConfig.tunnelEntranceToStartArea;
        Navigator.travelTo(tunnelEntrance.getCenter().getX(), tunnelEntrance.getCenter().getY(), true, true);   
        
        Heading toStartArea = CompetitionConfig.toStartAreaHeading;

        dll.localizeToTile(toStartArea, Board.getOrthogonalHeading(toStartArea), Board.getParallelHeading(toStartArea));
        travelThroughTunnel(CompetitionConfig.tunnelEntranceToSearchArea);
        dll.localizeToTile(toStartArea, Board.getOrthogonalHeading(toStartArea), toStartArea);
        
        Point2D start = Board.scTranslation[CompetitionConfig.corner];
        
        Navigator.travelTo(start.getX(), start.getY(), true, true);
        claw.release();
        
        Sound.beepSequenceUp();
        
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
        
        Point2D trans = Board.scTranslation[CompetitionConfig.corner];
        double rot = Board.scRotation[CompetitionConfig.corner];
        
        Odometer.getOdometer().setX(trans.getX());
        Odometer.getOdometer().setY(trans.getY());
        Odometer.getOdometer().setTheta(rot);
        
        // Compliance beeps
        for (int i = 0; i < 3; i++) {
            Sound.beep();
        }
        
       dll.travelToLine(100);
	}
	
	public static void travelToTunnel(DualLightLocalizer dll) throws OdometerExceptions {
	    
	    Tile tunnelEntrance = CompetitionConfig.tunnelEntranceToSearchArea;
	    Navigator.travelTo(tunnelEntrance.getCenter().getX(), tunnelEntrance.getCenter().getY(), true, true);   
	}
	

	/**
	 * Routine to travel to the tunnel
	 * 
	 * @param dll
	 * @throws OdometerExceptions
	 */
	/*
	private static void travelToTunnel(DualLightLocalizer dll) throws OdometerExceptions {
	    
	    // Get tunnel lower left and right
	    Tile toSearchArea = CompetitionConfig.tunnelEntranceToSearchArea;
        Tile tunnelUR = CompetitionConfig.tunnelEntranceToStartArea;
        
        // Debug information
        Log.log(Sender.Navigator, "tunnelLR: " + toSearchArea.toString());
        Log.log(Sender.Navigator, "tunnelUR: " + tunnelUR.toString());
        
        double targetX = toSearchArea.getLowerLeft().getX();
        double targetY = toSearchArea.getLowerLeft().getY();
        
        Navigator.travelTo(Odometer.getX(), targetY, true, true);
        dll.travelToLine(100);
        
        Navigator.travelSpecificDistance(5);
        
        Navigator.turnTo(Navigator.getDestAngle(targetX, toSearchArea.getCenter().getY()));
        dll.travelToLine(100);
        
        Navigator.travelTo(targetX, toSearchArea.getCenter().getY(), true, true);
        dll.travelToLine(100); 
        Navigator.travelSpecificDistance(5);
        
	}
	*/
	
	/**
	 * 
	 * @throws OdometerExceptions
	 */
	private static void travelThroughTunnel(Tile target) throws OdometerExceptions {
	            
        //Cheat the beginning.
        Navigator.travelSpecificDistance(Board.TILE_SIZE*2,(int) Vehicle.RIGHT_MOTOR.getMaxSpeed()/2);

        Vehicle.setMotorSpeeds(200, 200);
        while (!target.contains(Odometer.getX(), Odometer.getY())) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        
        Vehicle.setMotorSpeeds(0, 0);
        
        Navigator.travelSpecificDistance(Board.TILE_SIZE/2+5);

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