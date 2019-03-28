package ca.mcgill.ecse211.main;

import java.util.ArrayList;
import java.util.List;
import ca.mcgill.ecse211.util.*;

/**
 * Singleton system that creates and manages the polling thread. The primary directive of the poller system is to issue periodic update
 * events to subscribed {@link ca.mcgill.ecse211.util.Poller Poller} classes on the polling thread. 
 * {@link ca.mcgill.ecse211.util.Poller Poller}'s can subscribe to polling events by using {@link #addPoller(Poller poller)}. 
 * 
 * @author Brendan Kellam
 * @see ca.mcgill.ecse211.util.Poller
 */
public class PollerSystem implements Runnable {

    // Maximum amount of time (in ms) to wait between polling events
    private static final long POLLER_PERIOD = 40;
    
    // Static instance
    private static PollerSystem instance = null;
    
    private Thread pThread;
    private List<Poller> subscribedPollers = new ArrayList<Poller>();
    private boolean running = false;
    
    
    /**
     * Private constructor. Use {@link #getInstance()} to get a instance of {@link PollerSystem}
     */
    private PollerSystem() {
        this.pThread = new Thread(this);
    }
    
    /**
     * Get the instance of the poller system. This enforces the singleton pattern.
     * 
     * @return {@link PollerSystem} static instance
     */
    public static PollerSystem getInstance() {
        if (instance == null) {
            instance = new PollerSystem();
        }
        return instance;
    }
    
    
    /**
     * Implementation of {@link java.lang.Runnable#run()}. Runs on the polling thread, periodically issuing update events
     * to subscribed {@link ca.mcgill.ecse211.util.Poller Poller}.
     * 
     */
    @Override
    public void run() {        
        while (running) {
            
            long updateStart = System.currentTimeMillis();

            // Update all subscribed pollers
            for (Poller poller : subscribedPollers) {
                poller.update();
            }
            
            long updateEnd = System.currentTimeMillis();
            
            sleep(updateStart, updateEnd);
        }
        
    }
    
    private void sleep(long updateStart, long updateEnd) {
    	if (updateEnd - updateStart < POLLER_PERIOD) {
            try {
              Thread.sleep(POLLER_PERIOD - (updateEnd - updateStart));
            } catch (InterruptedException e) {
            }
          }
    }
    
    /**
     * Add a new poller to the subscribedPollers
     * 
     * @param poller
     */
    public void addPoller(Poller poller) {
        this.subscribedPollers.add(poller);
    }
    
    /**
     * Removes a poller from the subscribers list
     * 
     * @param index
     * @return {@link ca.mcgill.ecse211.util.Poller}
     * @throws IndexOutOfBoundsException
     */
    public Poller removePoller(int index) throws IndexOutOfBoundsException {
        return this.subscribedPollers.remove(index);
    }
    
    /**
     * Starts the polling thread
     * 
     * @throws RuntimeException
     */
    public void start() throws RuntimeException {
        if (running == true) {
            throw new RuntimeException("PollerSystem is already running.");
        }
        
        running = true;
        this.pThread.start();
    }
    
    /**
     * Stops the polling thread
     * 
     * @throws RuntimeException
     */
    public void stop() throws RuntimeException {
        if (running == false) {
            throw new RuntimeException("PollerSystem is not running.");
        }
        
        running = false;
    }
    
    
    /**
     * Return if the polling thread is running
     * 
     */
    public boolean isRunning() {
        return running;
    }
    

}
