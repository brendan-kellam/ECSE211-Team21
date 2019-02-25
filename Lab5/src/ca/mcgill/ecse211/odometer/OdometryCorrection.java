/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.util.Board;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	
 
  private static final long CORRECTION_PERIOD = 2;
  private Odometer odometer;
  
  private static final float LEFT_RGB_DELTA_THRESHOLD = 0.020f;
  private static final float RIGHT_RGB_DELTA_THRESHOLD = 0.025f;
  
  private static final int LINE_DETECTED_SLEEP_PERIOD = 4000;
  
  private boolean lineDetected = true;
  
  //private EV3ColorSensor colorSensor;
  
  private EV3ColorSensor leftCS;
  private EV3ColorSensor rightCS;

  private boolean correctionEnabled = false;
      
  /**
   * Normalized intensity readings from the R, G, B values
   */
  private float[] curRGBLeft;
  private float[] lastRGBLeft;
  
  private float[] curRGBRight;
  private float[] lastRGBRight;
  
  
  /**
   * Color sensor provider
   */
  private SampleProvider leftCSProvider;
  private SampleProvider rightCSProvider;
  
  private boolean leftDetected = false, rightDetected = false;
  

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
 * @param colorsensor 
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection(EV3ColorSensor leftCS, EV3ColorSensor rightCS) {
    try {
        this.odometer = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
        e.printStackTrace();
    }
    this.leftCS = leftCS;
    this.rightCS = rightCS;
            
    // Initialize colorSensor mode and buffer
    this.leftCSProvider = leftCS.getRGBMode();
    this.rightCSProvider = rightCS.getRGBMode();
    
    this.curRGBLeft = new float[leftCSProvider.sampleSize()];
    this.curRGBRight = new float[rightCSProvider.sampleSize()];

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
    this.lastRGBLeft = this.curRGBLeft.clone();
    this.lastRGBRight = this.curRGBRight.clone();
    leftDetected = false;
    rightDetected = false;
   
    while (true) {
      correctionStart = System.currentTimeMillis();

      if (correctionEnabled) {
                
          // Fetch the current RGB intensity sample
          leftCSProvider.fetchSample(curRGBLeft, 0);
          rightCSProvider.fetchSample(curRGBRight, 0);
          
          // If a line has already been detected
          if (lineDetected) {
              
              // Reset the last RGB to the current
              lastRGBLeft = curRGBLeft.clone();
              lastRGBRight = curRGBRight.clone();
              leftDetected = false;
              rightDetected = false;
              lineDetected = false;
          }
          
          leftDetected = detected(lastRGBLeft, curRGBLeft, LEFT_RGB_DELTA_THRESHOLD);
          rightDetected = detected(lastRGBRight, curRGBRight, RIGHT_RGB_DELTA_THRESHOLD);
          
          lastRGBLeft = curRGBLeft.clone();
          lastRGBRight = curRGBRight.clone();
          
          if (leftDetected && !rightDetected) {
              Sound.twoBeeps();
              
              int speed = Vehicle.LEFT_MOTOR.getSpeed();
              Vehicle.setMotorSpeeds(-speed, speed);
              
              while (!rightDetected) {
                  rightCSProvider.fetchSample(curRGBRight, 0);
                  rightDetected = detected(lastRGBRight, curRGBRight, RIGHT_RGB_DELTA_THRESHOLD);
                  lastRGBRight = curRGBRight.clone();
                  try {
                    Thread.sleep(CORRECTION_PERIOD);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
              }
              Sound.playTone(500, 500);
              Vehicle.setMotorSpeeds(speed, speed);
          }
          else if (!leftDetected && rightDetected) {
              
              Sound.twoBeeps();
              
              int speed = Vehicle.RIGHT_MOTOR.getSpeed();
              Vehicle.setMotorSpeeds(speed, -speed);

              while (!leftDetected) {
                  leftCSProvider.fetchSample(curRGBLeft, 0);
                  leftDetected = detected(lastRGBLeft, curRGBLeft, LEFT_RGB_DELTA_THRESHOLD);
                  lastRGBLeft = curRGBLeft.clone();
                  try {
                    Thread.sleep(CORRECTION_PERIOD);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
              }
              Sound.playTone(500, 500);
              Vehicle.setMotorSpeeds(speed, speed);
          }
          
          // If both left and right have been detected
          if (leftDetected && rightDetected) {
              double[] position = odometer.getXYT();
              Board.snapToGridLine(position, odometer);
              Sound.playTone(800, 1000);
              
              try {
                  Thread.sleep(LINE_DETECTED_SLEEP_PERIOD);
                  lineDetected = true;
              } catch (InterruptedException e) {
                  e.printStackTrace();
              }
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
  
  public void enableCorrection() {
      correctionEnabled = true;
  }
  
  public void disableCorrection() {
      correctionEnabled = false;
  }
  
  
}
