package ca.mcgill.ecse211.lab5;

import java.util.ArrayList;
import java.util.List;
import ca.mcgill.ecse211.util.*;

public class PollerSystem implements Runnable {

    
    private Thread pThread;
    
    private List<Poller> pollers = new ArrayList<Poller>();
    
    private boolean running = false;
    
    private static final long POLLER_PERIOD = 40; // odometer update period in ms
    
    public PollerSystem() {
        this.pThread = new Thread(this);
    }
    
    @Override
    public void run() {
        while (running) {
            
            long updateStart = System.currentTimeMillis();

            
            for (Poller poller : pollers) {
                poller.update();
            }
            
            long updateEnd = System.currentTimeMillis();
            
            if (updateEnd - updateStart < POLLER_PERIOD) {
              try {
                Thread.sleep(POLLER_PERIOD - (updateEnd - updateStart));
              } catch (InterruptedException e) {
              }
            }
        }
        
    }
    
    public void addPoller(Poller poller) {
        this.pollers.add(poller);
    }
    

    public void start() {
        running = true;
        this.pThread.start();
    }
    
    public void stop() {
        running = false;
    }
    
    public boolean isRunning() {
        return running;
    }
    

}
