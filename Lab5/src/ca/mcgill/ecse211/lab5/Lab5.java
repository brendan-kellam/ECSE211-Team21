package ca.mcgill.ecse211.lab5;

import java.io.FileNotFoundException;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.lab5.FieldSearch.SearchArea;
import ca.mcgill.ecse211.localization.FallingEdgeLocalizer;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.RisingEdgeLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Display;
import ca.mcgill.ecse211.util.Log;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.ArcAlgorithms;

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
        RISING_EDGE,
        FALLING_EDGE,
        INVALID
    }

    private static final float TILE_SIZE = 30.48f;

    
    /**
     * Main entry of program
     * @param args
     * @throws OdometerExceptions 
     */
    public static void main(String[] args) throws OdometerExceptions {
        
        
        // ----- Configuration ------
        
        // Create new board::
        
        
        // Lower left and upper right corner definitions [0,8]
        int LLx = 3, LLy = 3;
        int URx = 7, URy = 7;
        
        // Target can [1, 4]
        int TR = 3;
        
        // Starting corner
        FieldSearch.StartingCorner SC = FieldSearch.StartingCorner.LOWER_LEFT;
        
        // Starting corner [0, 3]
        FieldSearch.SearchArea searchArea = new FieldSearch.SearchArea(LLx, LLy, URx, URy, SC);
        
        
        
        /*
        // Create new vehicle configuration
        Vehicle.newConfig(new Vehicle.Configuration(2.1, 14.2));
        Vehicle.LEFT_MOTOR.setAcceleration(4000);
        Vehicle.RIGHT_MOTOR.setAcceleration(4000);
        
        Wheel wheel1 = WheeledChassis.modelWheel(Vehicle.LEFT_MOTOR, 4.3).offset(-7.2);
        Wheel wheel2 = WheeledChassis.modelWheel(Vehicle.RIGHT_MOTOR, 4.3).offset(7.2);
        Chassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
        
        MovePilot pilot = new MovePilot(chassis);
        pilot.setLinearSpeed(5);
        pilot.setLinearAcceleration(8);
        pilot.setAngularAcceleration(16);

        Navigator nav = new Navigator(pilot);
        
        nav.addWaypoint(TILE_SIZE, 0);
        
        nav.followPath();
        nav.waitForStop();
        
        
        System.out.println("x: " + nav.getPoseProvider().getPose().getX() + " | y: " + nav.getPoseProvider().getPose().getY());
        
                while (Button.waitForAnyPress() != Button.ID_ESCAPE);
        */
                
        // Create odometer
        Odometer odometer = Odometer.getOdometer();
        
        // Create ultrasonic poller
        UltrasonicPoller usPoller = new UltrasonicPoller(Vehicle.US_SENSOR);
        
        // Create new display object
        Display odometryDisplay = new Display(Vehicle.LCD_DISPLAY);
        
        // Create new UltrasonicLocalizer object
        //UltrasonicLocalizer ul = new UltrasonicLocalizer(usPoller);
        
        // Search the field
        FieldSearch fieldSearch = new FieldSearch(searchArea, SC, usPoller);
        
        // Initialize logging
        Log.setLogging(true, true, true, true);
        
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
        
        // Start ultrasonic poller thread 
        Thread usThread = new Thread(usPoller);
        usThread.start();
        
        // Start Display thread
        Thread odoDisplayThread = new Thread(odometryDisplay);
        odoDisplayThread.start();

        // Sleep to allow Display to initialize
        try {
            Thread.sleep(DISPLAY_INIT_SLEEP_TIME);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        Sound.twoBeeps();
        fieldSearch.startSearch();
        Sound.beep();
        
        /*
        // Execute US localization
        executeUSLocalization(usPoller, option);
        
        // Wait for user to continue
        while (Button.waitForAnyEvent() != Button.ID_ENTER);
        
        // Execute light sensor localization
        uc.localize();
        */
       
        // Wait
        while (Button.waitForAnyPress() != Button.ID_ESCAPE);
        System.exit(0);
    }
    
    /**
     * Executes ultrasonic localization
     * 
     * @param ul
     * @param option
     */
    private static void executeUSLocalization(UltrasonicPoller poller, MenuOption option) {
        UltrasonicLocalizer ul;
        if (option == MenuOption.FALLING_EDGE) {
            ul = new FallingEdgeLocalizer(poller);
        } else {
            ul = new RisingEdgeLocalizer(poller);
        }
        
        // Localize
        ul.localize();
    }
    
    /**
     * Gets the User's menu choice of either RISING or FALLING edge
     * 
     * @return MenuOption
     */
    public static MenuOption getUserChoice() {
        Vehicle.LCD_DISPLAY.drawString("Rising Edge  -> UP", 0, 0);
        Vehicle.LCD_DISPLAY.drawString("Falling Edge -> DOWN", 0, 1);
        
        int choice = Button.waitForAnyPress();
        
        if (choice == Button.ID_UP) {
            return MenuOption.RISING_EDGE;
        }
        else if (choice == Button.ID_DOWN) {
            return MenuOption.FALLING_EDGE;
        }
        
        return MenuOption.INVALID;
    }
    
}
