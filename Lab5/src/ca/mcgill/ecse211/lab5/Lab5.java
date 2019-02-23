package ca.mcgill.ecse211.lab5;

import java.io.FileNotFoundException;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.localization.FallingEdgeLocalizer;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.RisingEdgeLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.util.Display;
import ca.mcgill.ecse211.util.Log;
import lejos.hardware.Button;

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
    
    private enum StartingCorner {
        LOWER_LEFT,
        LOWER_RIGHT,
        UPPER_RIGHT,
        
    }
    
    /**
     * Main entry of program
     * @param args
     * @throws OdometerExceptions 
     */
    public static void main(String[] args) throws OdometerExceptions {
        
        
        // ----- Configuration ------
        
        // Lower left and upper right corner definitions [0,8]
        int LLx, LLy;
        int URx, URy;
        
        // Target can [1, 4]
        int TR;
        
        // Starting corner [0, 3]
        int SC;
        
        // Create new vehicle configuration
        Vehicle.newConfig(new Vehicle.Configuration(2.1, 13.5));
        Vehicle.LEFT_MOTOR.setAcceleration(4000);
        Vehicle.RIGHT_MOTOR.setAcceleration(4000);
        
        // Create odometer
        Odometer odometer = Odometer.getOdometer();
        
        // Create ultrasonic poller
        UltrasonicPoller usPoller = new UltrasonicPoller(Vehicle.US_SENSOR);
        
        // Create new display object
        Display odometryDisplay = new Display(Vehicle.LCD_DISPLAY);
        
        // Create new UltrasonicLocalizer object
        //UltrasonicLocalizer ul = new UltrasonicLocalizer(usPoller);
        
        // Create new LightLocalizer
        LightLocalizer uc = new LightLocalizer();
        
        // Initialize logging
        Log.setLogging(true, true, true, true);
        
        // Set logging to write to file
        try {
            Log.setLogWriter("Lab4" + ".log");
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
        
        // Execute US localization
        executeUSLocalization(usPoller, option);
        
        // Wait for user to continue
        while (Button.waitForAnyEvent() != Button.ID_ENTER);
        
        // Execute light sensor localization
        uc.localize();
        
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
