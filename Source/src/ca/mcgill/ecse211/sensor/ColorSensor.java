package ca.mcgill.ecse211.sensor;

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
    
}
