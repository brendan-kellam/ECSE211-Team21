package ca.mcgill.ecse211.hardware;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * Encapsulates all hardware elements of the Vehicle into a single API
 *
 */
public final class Vehicle {
    
    //////////////// MOTORS ////////////////
    public static final EV3LargeRegulatedMotor LEFT_MOTOR =
        new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    
    public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
        new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    ///////////////////////////////////////
    
    //////////////// LCD  ////////////////
    public static final TextLCD LCD_DISPLAY = LocalEV3.get().getTextLCD();
    ///////////////////////////////////////

    ////////////////ULTRASONIC ////////////////
    public static final EV3UltrasonicSensor US_SENSOR = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
    ////////////////////////////////////////////
    
    ////////////////COLOR ////////////////
    public static final EV3ColorSensor LEFT_COLOR_SENSOR = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
    public static final EV3ColorSensor RIGHT_COLOR_SENSOR = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
    //////////////////////////////////////
    
    
    
    // Configuration class
    private static Configuration configuration;
    
    /**
     * Create a new configuration for the Vehicle
     * 
     * @param configuration
     */
    public static void newConfig(Configuration configuration) {
        Vehicle.configuration = configuration;
    }
    
    /**
     * Get assigned configuration
     * 
     * @return configuration
     */
    public static Configuration getConfig() {
        return configuration;
    }
    
    
    /**
     * Represents the current configuration utilized on the Vehicle
     */
    public static class Configuration {
        
        // Wheel radius
        private double wheelRad;
        
        // Track width (cm)
        private double trackWidth;
        
        /**
         * Default constructor for a new configuration. <br>
         * <b>TODO:</b> Consider using the builder pattern to allow dynamic configurations for optional parameters <br>
         * <b>TODO:</b> In the future, "wheelRad" may need to be abstracted out into a separate class. For competition, we don't know the number of wheels,
         * their radiuses or if we are even using wheels at all (track configuration)
         * 
         * @param wheelRad
         * @param trackWidth
         */
        public Configuration(double wheelRad, double trackWidth) {
            this.wheelRad = wheelRad;
            this.trackWidth = trackWidth;
        }
        
        /**
         * Getter for vehicle's wheel radius
         * 
         * @return wheelRad
         */
        public double getWheelRadius() {
            return this.wheelRad;
        }
        
        /**
         * Getter for vehicle's track width
         * 
         * @return trackWidth
         */
        public double getTrackWidth() {
            return this.trackWidth;
        }
    }
    
    
    /**
     * Set the left and right speeds for LEFT_MOTOR and RIGHT_MOTOR respectively
     * 
     * @param leftSpeed
     * @param rightSpeed
     */
    public static <T extends Number> void setMotorSpeeds(T leftSpeed, T rightSpeed) {
      LEFT_MOTOR.setSpeed(leftSpeed.floatValue());
      RIGHT_MOTOR.setSpeed(rightSpeed.floatValue());
      
      // Set left motor rotation direction
      if (leftSpeed.floatValue() < 0) {
        LEFT_MOTOR.backward();
      } else {
        LEFT_MOTOR.forward();
      }
      
      // Set right motor rotation direction
      if (rightSpeed.floatValue() < 0) {
        RIGHT_MOTOR.backward();
      } else {
        RIGHT_MOTOR.forward();
      }
    }
    
}
