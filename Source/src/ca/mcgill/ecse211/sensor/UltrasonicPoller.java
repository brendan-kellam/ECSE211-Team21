package ca.mcgill.ecse211.sensor;

import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.util.Poller;
import lejos.robotics.SampleProvider;


public class UltrasonicPoller implements Poller {
  private SampleProvider us;
  private float[] usData;
  int distance;

  public UltrasonicPoller(SampleProvider us) {
    this.us = us;
    usData = new float[us.sampleSize()];
  }
  
  
    @Override
    public void update() {
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

    public int getDistance() {
        return distance;
      }

}
