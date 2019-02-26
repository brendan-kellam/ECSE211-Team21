package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Board.Heading;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * Implements light localization
 */
public class LightLocalizer implements Localizer {
    
    // RGB threashold
    private static final float LEFT_RGB_DELTA_THRESHOLD = 0.018f;
    private static final float RIGHT_RGB_DELTA_THRESHOLD = 0.026f;
    
    
    // Is line detected
    private boolean leftDetected = false, rightDetected = false;

    /**
     * Color sensor provider
     */
    private SampleProvider leftCSProvider;
    private SampleProvider rightCSProvider;
    
    private float[] curRGBLeft;
    private float[] lastRGBLeft;
    
    private float[] curRGBRight;
    private float[] lastRGBRight;
        
    // X and Y grid line diffs
    private static double X_DIFF = 11.5;
    private static double Y_DIFF = 9;
    
    // Board
    private double finalX, finalY;
    
    // Light sensor that had a detection
    private enum DetectionType {
        NONE,
        LEFT,
        RIGHT,
        BOTH
    }
    
    public LightLocalizer(double finalX, double finalY) {
        // Initialize colorSensor mode and buffer
        this.leftCSProvider = Vehicle.LEFT_COLOR_SENSOR.getRGBMode();
        this.rightCSProvider = Vehicle.RIGHT_COLOR_SENSOR.getRGBMode();
        
        this.curRGBLeft = new float[leftCSProvider.sampleSize()];
        this.curRGBRight = new float[rightCSProvider.sampleSize()];

        
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
        // Clone the current rgb reading
        this.lastRGBLeft = this.curRGBLeft.clone();
        this.lastRGBRight = this.curRGBRight.clone();
        leftDetected = false;
        rightDetected = false;
        
        Vehicle.LEFT_MOTOR.setAcceleration(6000);
        Vehicle.RIGHT_MOTOR.setAcceleration(6000);
        
        // Detect first x grid line
        double y1 = Odometer.getY();
        driveTillLineDetection(100, 5, true, DetectionType.LEFT);
        wait(200);
        double y2 = Odometer.getY();
        
        // Reverse backwards to y1
        try {
            Navigator.travelTo(Odometer.getX(), y1, true, false, -100);
        } catch (OdometerExceptions e3) {
            e3.printStackTrace();
        }
        wait(200);
        
        // Turn 90 degrees clockwise
        Navigator.turnTo(90.0); 
        wait (200);
        
        // Drive to next grid line
        driveTillLineDetection(100, 10, true, DetectionType.LEFT);
        wait(200);
        double x2 = Odometer.getX();
        
        Sound.beep();
            
        // Reverse
        Vehicle.setMotorSpeeds(-100, -100);
        while (Math.abs(Odometer.getX() - x2+X_DIFF) > 1.5) {
            wait(30);
        }
        Vehicle.setMotorSpeeds(0, 0);

        // Turn counter clockwise
        wait(200);
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
    
    private void driveTillLineDetection(int speed, int sleep, boolean stop, DetectionType type) {
        Vehicle.setMotorSpeeds(speed, speed);
        
        leftCSProvider.fetchSample(curRGBLeft, 0);
        rightCSProvider.fetchSample(curRGBRight, 0);
        // Update last RGB values
        lastRGBLeft = curRGBLeft.clone();
        lastRGBRight = curRGBRight.clone();
        
        DetectionType t;
        boolean leftDetected = false;
        boolean rightDetected = false;
        
        // Continue while
        while ((t = lineDetected()) != DetectionType.BOTH) {
                        
            if (t == DetectionType.RIGHT) {
                Vehicle.RIGHT_MOTOR.setSpeed(0);
                Sound.playTone(400, 500);
                leftDetected = true;
            } else if (t == DetectionType.LEFT) {
                Vehicle.LEFT_MOTOR.setSpeed(0);
                Sound.playTone(800, 500);
                rightDetected = true;
            }
            
            if ( leftDetected && rightDetected) {
                break;
            }
            
            try {
                Thread.sleep(sleep);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        
        Sound.beepSequenceUp();
        
        Heading heading = Board.getHeading(Odometer.getTheta());
        
        if (heading == Heading.N) {
            try {
                Odometer.getOdometer().setTheta(0.0);
            } catch (OdometerExceptions e) {
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
    private DetectionType lineDetected() {
     // Fetch the current RGB intensity sample
        leftCSProvider.fetchSample(curRGBLeft, 0);
        rightCSProvider.fetchSample(curRGBRight, 0);
        
        if (leftDetected) {
            
            // Reset the last RGB to the current
            lastRGBLeft = curRGBLeft.clone();
            leftDetected = false;
        }
        
        // If a line has already been detected
        if (rightDetected) {
            
            // Reset the last RGB to the current
            lastRGBRight = curRGBRight.clone();
            rightDetected = false;
        }
        
        leftDetected = detected(lastRGBLeft, curRGBLeft, LEFT_RGB_DELTA_THRESHOLD);
        rightDetected = detected(lastRGBRight, curRGBRight, RIGHT_RGB_DELTA_THRESHOLD);
        
        // Update last RGB values
        lastRGBLeft = curRGBLeft.clone();
        lastRGBRight = curRGBRight.clone();
        
        if (leftDetected && rightDetected) {
            return DetectionType.BOTH;
        } else if (leftDetected && !rightDetected) {
            return DetectionType.LEFT;
        } else if (!leftDetected && rightDetected) {
            return DetectionType.RIGHT;
        }
        
        return DetectionType.NONE;
    }
    
    private boolean detected(float[] last, float[] cur, double threshold) {
        // Get RGB components for last reading and currect
        float r1 = last[0];
        float g1 = last[1];
        float b1 = last[2];
        
        float r2 = cur[0];
        float g2 = cur[1];
        float b2 = cur[2];
        
        // Compute difference in each component
        float rdiff = r2 - r1;
        float gdiff = g2 - g1;
        float bdiff = b2 - b1;
        
        // Compute delta between the r, g, b components of the last reading and the current reading
        float delta = (float) Math.sqrt(rdiff*rdiff + gdiff*gdiff + bdiff*bdiff);
        
        return delta > threshold;
    }
    
}
