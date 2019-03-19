package ca.mcgill.ecse211.ultrasonic;

import ca.mcgill.ecse211.util.Log;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;


public class UltrasonicPoller implements Runnable {
  private SampleProvider us;
  private float[] usData;
  int distance;
  volatile boolean exit = false;

  public UltrasonicPoller(SampleProvider us) {
    this.us = us;
    usData = new float[us.sampleSize()];
  }

  // Sensors now return floats using a uniform protocol.
  // Need to convert US result to an integer [0,255]

  public void run() {
    while (!exit) {
      us.fetchSample(usData, 0); // acquire data
      distance = (int) (usData[0] * 100.0); // extract from buffer, cast
                          // to int

      Log.log(Log.Sender.usSensor, Integer.toString(distance));

      try {
        Thread.sleep(30);
      } catch (Exception e) {
        e.printStackTrace();
      } 
    }
    Sound.buzz();
  }

  public int getDistance() {
    return distance;
  }

  public void stop() {
	  exit = true;
  }
}
