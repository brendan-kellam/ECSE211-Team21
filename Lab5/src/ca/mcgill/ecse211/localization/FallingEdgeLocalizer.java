package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.util.Log;

/**
 * Falling edge ultrasonic localizer
 */
public class FallingEdgeLocalizer extends UltrasonicLocalizer {

    
    /**
     * Default constructor. Calls super constructor
     * 
     * @param usPoller
     */
    public FallingEdgeLocalizer(UltrasonicPoller usPoller) {
        super(usPoller);
    }


    /**
     * Detects if the vehicle is in the falling edge noise margin
     */
    @Override
    protected boolean inNoiseMargin(double distance) {
        return distance < DETECTION_DISTANCE + NOISE_MARGIN;
    }


    /**
     * Detects if the vehicle is in the falling edge detection margin
     */
    @Override
    protected boolean inDetectionMargin(double distance) {
        return distance < DETECTION_DISTANCE - NOISE_MARGIN;
    }

    /**
     * Computes new heading given midAngle
     */
    @Override
    protected double computeNewHeading(double midAngle) {
        
        double dTheta = 45 - midAngle;
        if (dTheta < 0) {
            dTheta+=180;
        }
        
        Log.log(Log.Sender.USLocalization, "dTheta = " + dTheta);

        
        double newHeading = Odometer.getTheta()+dTheta;
        newHeading = (newHeading > 360.0) ? newHeading-180 : newHeading;
        
        return newHeading;
    }

}
