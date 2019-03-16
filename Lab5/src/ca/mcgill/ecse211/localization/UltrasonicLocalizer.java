package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.util.EV3Math;
import ca.mcgill.ecse211.util.Log;
import lejos.hardware.Sound;

/**
 * Base class for ultrasonic localizers
 */
public abstract class UltrasonicLocalizer implements Localizer {
    
    // Motor speeds
    private static int SPEED = 100; 
    
    // Ultrasonic poller
    private UltrasonicPoller usPoller;
    
    // If localization has completed
    private boolean localizationComplete;
    
    // Noise margin
    protected static final double NOISE_MARGIN = 1.0;
    
    // Detection distance
    protected static final double DETECTION_DISTANCE = 34;
    
    // Odometer angle for INMARGIN event
    private double theta1 = 0.0;
    
    // Odometer angle for DETECTED event
    private double theta2 = 0.0;
    
    // Amount of time to wait after detection (Prevents a wall being detected twice)
    private static final int WAIT_TIME_AFTER_DETECTION = 230;
    
    
    /**
     * States of the US localizer
     */
    private enum State {
        TURN,
        WAIT,
        SCAN,
        INMARGIN,
        DETECTED
    }
    
    // Current state of US localizer
    private State state;
    
    
    /**
     * 
     * @param usPoller
     */
    public UltrasonicLocalizer(UltrasonicPoller usPoller) {
        this.usPoller = usPoller;
    }
    
    /**
     * 
     * @param newState
     */
    protected void switchState(State newState) {
        Log.log(Log.Sender.USLocalization, "State change: (" + this.state.toString() + ") -> (" + newState.toString() + ")");
        this.state = newState;
    }
    
    
    /**
     * Perform localization operation
     */
    public void localize() {
        
        // Reset localizationComplete boolean and set state to TURN
        this.localizationComplete = false;
        state = State.TURN;
        
        // Speed of rotation
        int speed = SPEED;
        
        // Alpha and beta fields
        double alpha = -1.0, beta = -1.0;
        
        // Distance
        int distance;
        
        // Cycles to wait following wall detection
        int waitCount = 0; 
        
        // Go until localization is complete
        while (!localizationComplete) {
            
            // FSM
            switch (state) {
            
            ///////////////TURN STATE/////////////////
            case TURN:
            {
                Vehicle.setMotorSpeeds(speed, -speed);
                switchState(State.WAIT);
                break;
            }
            
            ///////////////WAIT STATE/////////////////
            case WAIT:
            {
                if (waitCount == 0) {
                    switchState(State.SCAN);
                    Sound.beep();
                } else {
                    waitCount--;
                }
                break;
            }
            
            ///////////////DETECT STATE/////////////////
            case SCAN:
            {
                distance = usPoller.getDistance();
                
                // Has entered noise margin
                if (inNoiseMargin(distance)) {
                    
                    // Set first theta
                    theta1 = Odometer.getTheta();
                    
                    // Switch to INMARGIN state
                    switchState(State.INMARGIN);
                }
                break;
            }
            
            ///////////////IN MARGIN STATE/////////////////
            case INMARGIN:
            {
                distance = usPoller.getDistance();
                
                if (inDetectionMargin(distance)) {
                    switchState(State.DETECTED);
                } else if (!inNoiseMargin(distance)) {
                    switchState(State.SCAN);
                }
                break;
            }
            
            ///////////////DETECTION STATE/////////////////
            case DETECTED:
            {
                // Set theta2 and compute average across two detection events
                theta2 = Odometer.getTheta();
                double theta = (theta1 + theta2) / 2;
                
                // Log results and beep
                Log.log(Log.Sender.USLocalization, "Theta1 = " + theta1 + " | Theta2 = " + theta2 + " | Avg = " + theta);
                Sound.beep();
                
                // Swap rotation
                speed = -speed;
               
                // Set wait time
                waitCount = WAIT_TIME_AFTER_DETECTION;
                
                // First detection event
                if (alpha == -1.0) {
                    
                    // Set alpha and swap back to turn state
                    alpha = theta;
                    switchState(State.TURN);
                    
                // Second detection event
                } else {
                    
                    // Localization is now complete
                    localizationComplete = true;
                    
                    // Set beta reading
                    beta = theta;
                    Log.log(Log.Sender.USLocalization, "Alpha = " + alpha + " | Beta = " + beta);
                    
                    double midAngle = EV3Math.boundAngle(alpha + beta) / 2.0;
                    
                    Log.log(Log.Sender.USLocalization, "midAngle = " + midAngle);
                    
                    // Get new heading
                    double newHeading = computeNewHeading(midAngle);
                    
                    Log.log(Log.Sender.USLocalization, "Theta = " + Odometer.getTheta());
                    Log.log(Log.Sender.USLocalization, "newHeading = " + newHeading);
                    
                    try {
                        Odometer.getOdometer().setTheta(newHeading);
                    } catch (OdometerExceptions e) {
                        e.printStackTrace();
                    }
                                        
                    Log.log(Log.Sender.USLocalization, "new Theta = " + Odometer.getTheta());
                               
                    // Turn to 0.0
                    Navigator.turnTo(0.0);
                }
                
                break;
            }
                
            }
            
            // Sleep thread
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        
        
        
    }
    
    /**
     * 
     * @param midAngle
     * @return
     */
    protected abstract double computeNewHeading(double midAngle);
    
    /**
     * 
     * @param distance
     * @return
     */
    protected abstract boolean inNoiseMargin(double distance);
    
    /**
     * 
     * @param distance
     * @return
     */
    protected abstract boolean inDetectionMargin(double distance);
    
}
