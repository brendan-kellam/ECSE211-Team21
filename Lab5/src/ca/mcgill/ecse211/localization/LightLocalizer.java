package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * Implements light localization
 */
public class LightLocalizer implements Localizer {
    
    /**
     * Normalized intensity readings from the R, G, B values
     */
    private float[] curRGB;
    private float[] lastRGB;
    
    /**
     * Color sensor provider
     */
    private SampleProvider csProvider;
    
    // RGB threashold
    private static final float RGB_DELTA_THRESHOLD = 0.015f;
    
    // Is line detected
    private boolean lineDetected = true;
        
    // X and Y grid line diffs
    private static double X_DIFF = 13.5;
    private static double Y_DIFF = 12.5;
    
    private double finalX;
    private double finalY;

    
    public LightLocalizer(double finalX, double finalY) {
        // Initialize colorSensor mode and buffer
        this.csProvider = Vehicle.RIGHT_COLOR_SENSOR.getRGBMode();
        this.curRGB = new float[csProvider.sampleSize()];
        
        this.finalX = finalX;
        this.finalY = finalY;
    }

    /**
     * Perform localization routine.
     * 
     * The vehicle will drive up to the first grid line, recording it's offset from it's starting location.
     * Upon reaching the grid line, it will reverse back to it's original position.
     * 
     * The vehicle will then rotate 90 degrees clockwise and repeat the same procedure above for the second grid line.
     * 
     * The vehicle will then reverse a set distance backward based of the second difference measured, 
     * turn 90 degrees counter-clockwise and drive forward a set distance based of the first difference measured.
     */
    public void localize() {
        this.lastRGB = this.curRGB;
        
        // Detect first x grid line
        double y1 = Odometer.getY();
        driveTillLineDetection(100, 50, true);
        double y2 = Odometer.getY();
        wait(1000);
        
        // Reverse backwards to y1
        try {
            Navigator.travelTo(Odometer.getX(), y1, true, false, -100);
        } catch (OdometerExceptions e3) {
            e3.printStackTrace();
        }
        wait(1000);
        
        // Turn 90 degrees clockwise
        Navigator.turnTo(90.0); 
        wait (1000);
        
        // Drive to next grid line
        driveTillLineDetection(100, 50, true);
        double x2 = Odometer.getX();
            
        // Reverse
        Vehicle.setMotorSpeeds(-100, -100);
        while (Math.abs(Odometer.getX() - x2+X_DIFF) > 1.5) {
            wait(30);
        }
        Vehicle.setMotorSpeeds(0, 0);

        // Turn counter clockwise
        wait(1000);
        Navigator.turnTo(0.0);
        
        // Forward to origin
        Vehicle.setMotorSpeeds(100, 100);
        while (Math.abs(Odometer.getY() - y2+Y_DIFF) > 1.5) {
            wait(30);
        }
        Vehicle.setMotorSpeeds(0, 0);
        
        // Reset odometer values
        try {
            Odometer.getOdometer().setX(finalX);
            Odometer.getOdometer().setY(finalY);
        } catch (OdometerExceptions e) {
            e.printStackTrace();
        }
        
        
    }
    
    private void driveTillLineDetection(int speed, int sleep, boolean stop) {
        Vehicle.setMotorSpeeds(speed, speed);
        
        // Continue while
        while (!lineDetected()) {
            try {
                Thread.sleep(sleep);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        
        if (stop) {
            Vehicle.setMotorSpeeds(0, 0);
        }
    }
    
    /**
     * Sleep thread wait ms
     */
    private void wait(int wait) {
        try {
            Thread.sleep(wait);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    /**
     * 
     * @return
     */
    private boolean lineDetected() {
        // Fetch the current RGB intensity sample
        csProvider.fetchSample(curRGB, 0);
        
        // If a line has already been detected
        if (lineDetected) {
            
            // Reset the last RGB to the current
            lastRGB = curRGB.clone();
            lineDetected = false;
        }
        
        // Get RGB components for last reading and currect
        float r1 = lastRGB[0];
        float g1 = lastRGB[1];
        float b1 = lastRGB[2];
        
        float r2 = curRGB[0];
        float g2 = curRGB[1];
        float b2 = curRGB[2];
        
        // Compute difference in each component
        float rdiff = r2 - r1;
        float gdiff = g2 - g1;
        float bdiff = b2 - b1;
        
        // Compute delta between the r, g, b components of the last reading and the current reading
        float delta = (float) Math.sqrt(rdiff*rdiff + gdiff*gdiff + bdiff*bdiff);
        
        // update last RGB value
        lastRGB = curRGB.clone();
        
        // If color delta is significant
        if (delta > RGB_DELTA_THRESHOLD) {
            lineDetected = true;
            Sound.beep();
            return true;
        }
        return false;
    }
    
}
