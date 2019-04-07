package ca.mcgill.ecse211.sensor;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.util.Tile;
import ca.mcgill.ecse211.util.Vehicle;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * Wraps a {@link lejos.hardware.sensor.EV3ColorSensor} with additional functionality like line detection and
 * differential filters.
 */
public class ColorSensor {

    
    /**
     * Threshold for this Color sensor
     */
    private float lineThreshold;
    
    /**
     * Normalized intensity readings from the R, G, B values
     */
    private SensorMode sensorColour;
    
    /**
     * The wrapped color sensor
     */
    private EV3ColorSensor sensor;
    
    /**
     * Default constructor
     * 
     * @param sensor - The color sensor to wrap
     * @param lineThreshold - The threshold value this color sensor to accurately detect a line 
     */
    public ColorSensor(EV3ColorSensor sensor, float lineThreshold) {
        this.sensor = sensor;
        this.lineThreshold = lineThreshold;
        sensorColour = sensor.getRedMode();
    }
    
    /**
     * Returns true if a line has been detected. Uses a comparison between {@link #fetchNormalizedSample()} and
     * {@link #lineThreshold} to determine.
     * 
     * @return isLineDetected
     */
    public boolean lineDetected() {
        //Get the current intensity. On the blue board, the value is roughly 350.
        float curIntensity = fetchNormalizedSample();
        if (curIntensity < lineThreshold) {
            return true;
        }
        return false;
    }
    
    /**
     * Fetch a new sample: one element containing the intensity level (Normalized between 0 and 1) of reflected light.
     * 
     * @return normalized intensity
     */
    private float fetchNormalizedSample() {
        float[] intensity = new float[sensorColour.sampleSize()];
        sensorColour.fetchSample(intensity, 0);
        return intensity[0];
    }
    
    /**
     * Get the x position
     * 
     * @param sensor
     * @return
     */
    public static double getX(ColorSensor sensor) throws IllegalArgumentException {
        
        double angle = Math.toRadians(Odometer.getTheta());
        
        double x = Odometer.getX() - Vehicle.VERT_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE * Math.sin(angle);
        
        if (sensor == Vehicle.LEFT_CS) { 
            x -= Vehicle.HORZ_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE * Math.cos(angle);
            return x;
            
            
        } else if (sensor == Vehicle.RIGHT_CS) {
            x += Vehicle.HORZ_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE * Math.cos(angle);
            return x;
        }
        
        throw new IllegalArgumentException("sensor is neither left or right mounted. Unable to get position.");
    }
    
    /**
     * Get the y position
     * 
     * @param sensor
     * @return
     */
    public static double getY(ColorSensor sensor) throws IllegalArgumentException {
        
        double angle = Math.toRadians(Odometer.getTheta()); 
        double y = Odometer.getY() - Vehicle.VERT_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE * Math.cos(angle);
        
        if (sensor == Vehicle.LEFT_CS) {
            
            y += Vehicle.HORZ_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE * Math.sin(angle);
            
            return  y;
        } else if (sensor == Vehicle.RIGHT_CS) {
            
            y -= Vehicle.HORZ_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE * Math.sin(angle);
            
            return y;
        }
        
        throw new IllegalArgumentException("sensor is neither left or right mounted. Unable to get position.");
    }
    
    /**
     * Returns true if a tile contains a given sensor
     * 
     * @return
     */
    public static boolean checkTileContains(ColorSensor sensor, Tile tile) {
        return tile.contains(getX(sensor), getY(sensor));
    }
    
}
