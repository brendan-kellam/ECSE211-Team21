package ca.mcgill.ecse211.util;

/**
 *  This interface enables the implementing class to subscribe to polling event from the polling thread.
 *  Polling events are dispatched via the class's update method.
 *  
 *  @see ca.mcgill.ecse211.main.PollerSystem
 *  @author Brendan Kellam
 */
public interface Poller {
   
    
    /**
     * Performs a discrete polling operation. If the implementing class subscribes to the PollerSystem's polling list,
     * the method will be called periodically from the poller thread.
     * 
     * @see ca.mcgill.ecse211.main.PollerSystem
     */
    public void update();
}
