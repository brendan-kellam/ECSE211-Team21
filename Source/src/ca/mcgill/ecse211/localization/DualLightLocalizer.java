package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.light.ColorSensor;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Board.Heading;
import lejos.hardware.Sound;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Blocking routine to 
 */
public class DualLightLocalizer {

    
    /**
     * Left and right color sensors
     */
    private ColorSensor leftSensor;
    private ColorSensor rightSensor;
    
    /**
     * Default constructor. Accepts two {@link ca.mcgill.ecse211.light.ColorSensor ColorSensor} objects representing the
     * left and right mounted color sensors.
     * 
     * @param leftSensor
     * @param rightSensor
     */
    public DualLightLocalizer(ColorSensor leftSensor, ColorSensor rightSensor) {
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;
    }
    
    /**
     * Localize to a line
     */
    public void localize(Heading heading) {
        
        // Start by turning to the given heading
        Navigator.turnTo(Board.getHeadingAngle(heading));
        
        LineRunner l = new LineRunner(leftSensor, Vehicle.LEFT_MOTOR, 100);
        LineRunner r = new LineRunner(rightSensor, Vehicle.RIGHT_MOTOR, 100);
        
        Thread lThread = new Thread(l);
        Thread rThread = new Thread(r);
        
        lThread.start();
        rThread.start();
        
        try {
            lThread.join();
            rThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        
        
        /*
        boolean left = false, right = false; 
        
        // Begin by advancing forward until either the left or right sensor detects a line
        Vehicle.setMotorSpeeds(50, 50);
        while (!( (left = leftSensor.lineDetected()) || (right = rightSensor.lineDetected()) )) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        Sound.twoBeeps();

        // If both sensors detected a line
        if (left && right) {
            Vehicle.setMotorSpeeds(0, 0);
        }
        
        // Left detected
        else if (left) {
            Vehicle.LEFT_MOTOR.stop();
            while (!rightSensor.lineDetected()) {
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            Vehicle.RIGHT_MOTOR.stop();
        }
        
        // right detected
        else if (right) {
            Vehicle.RIGHT_MOTOR.stop();
            while (!leftSensor.lineDetected()) {
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            Vehicle.LEFT_MOTOR.stop();
        }
        
        */
        
    }

    class LineRunner implements Runnable {
        
        private ColorSensor sensor;
        private EV3LargeRegulatedMotor motor;
        private float speed;
        
        public LineRunner(ColorSensor sensor, EV3LargeRegulatedMotor motor, float speed) {
            this.sensor = sensor;
            this.motor = motor;      
            this.speed = speed;
        }

        @Override
        public void run() {
            
            motor.setSpeed(speed);
            motor.forward();
            
            while (!sensor.lineDetected()) {
                try {
                    Thread.sleep(30);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            
            motor.stop();
            
            Sound.beep();
        }
        
    }
    
}
