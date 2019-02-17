/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.util.Board;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	
 
  private static final long CORRECTION_PERIOD = 2;
  private Odometer odometer;
  
  private static final float RGB_DELTA_THRESHOLD = 0.015f;
  private static final int LINE_DETECTED_SLEEP_PERIOD = 1000;
  
  private boolean lineDetected = true;
  
  private EV3ColorSensor colorSensor;
    
  private Board board;
  
  /**
   * Normalized intensity readings from the R, G, B values
   */
  private float[] curRGB;
  private float[] lastRGB;
  
  /**
   * Color sensor provider
   */
  private SampleProvider csProvider;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
 * @param colorsensor 
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection(EV3ColorSensor colorSensor, Board board) throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    this.colorSensor = colorSensor;
            
    // Initialize colorSensor mode and buffer
    this.csProvider = colorSensor.getRGBMode();
    this.curRGB = new float[csProvider.sampleSize()];
    
    // Create a new board
    this.board = board;
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    
    // Clone the current rgb reading
    this.lastRGB = this.curRGB.clone();
   
    while (true) {
      correctionStart = System.currentTimeMillis();
            
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
    	      	      	  
    	  double[] position = odometer.getXYT();
    	  board.snapToGridLine(position, odometer);
          Sound.playTone(800, 100);
    	  
    	try {
			Thread.sleep(LINE_DETECTED_SLEEP_PERIOD);
			lineDetected = true;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
      }
      
      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
  
  
  
}
