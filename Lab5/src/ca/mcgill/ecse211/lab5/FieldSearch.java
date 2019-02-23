package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import lejos.robotics.geometry.Point;

public class FieldSearch {

    
    // Ultrasonic poller
    private UltrasonicPoller usPoller;
    
    // Search area
    private SearchArea searchArea;
    
    private StartingCorner startingCorner;
    
    private static final double TILE_SIZE = 30.48;
    
    public FieldSearch(SearchArea searchArea, StartingCorner SC, UltrasonicPoller poller) {
        this.usPoller = poller;
        this.searchArea = searchArea;
        this.startingCorner = SC;
    }
    
    /**
     * Method shall::
     * 
     * 
     */
    public void startSearch() {
        
        // Navigate to lower left corner
        try {
            navigateToLL();
        } catch (OdometerExceptions e) {
            e.printStackTrace();
        }
        
        // Turn to face N
        Navigator.turnTo(0.0, true);
        
        // Traveling to bottom left
        Point bottomLeft = searchArea.getBottomLeft();
        
        // Localize to field
        LightLocalizer ll = new LightLocalizer(bottomLeft.getX()*TILE_SIZE, bottomLeft.getY()*TILE_SIZE);
        ll.localize();
        
        // Navigate to center of lower left tile
        try {
            Navigator.travelTo(bottomLeft.getX()*TILE_SIZE + TILE_SIZE/2, bottomLeft.getY()*TILE_SIZE + TILE_SIZE/2, true, true);
        } catch (OdometerExceptions e) {
            e.printStackTrace();
        }
        Navigator.turnTo(0.0, true);
        
    }
    
    
    /**
     * Navigate to the lower left-hand corner of the search area
     * @throws OdometerExceptions 
     */
    private void navigateToLL() throws OdometerExceptions {
        
        
        // Traveling to bottom left
        Point bottomLeft = searchArea.getBottomLeft();
        
        double targetX = (bottomLeft.getX() - 1) * TILE_SIZE + TILE_SIZE/1.5;
        double targetY = (bottomLeft.getY() - 1) * TILE_SIZE + TILE_SIZE/1.5;
        
        // Navigate to lower left
        Navigator.travelTo(targetX, targetY, true, true);
    }
    
    
    static class SearchArea {
                
        private Point bottomLeft, bottomRight, topLeft, topRight;
        
        private StartingCorner startingCorner;
        
        public SearchArea(int LLx, int LLy, int URx, int URy, StartingCorner SC) {
            
            this.startingCorner = SC;

            Point bl = new Point(LLx, LLy);
            Point br = new Point(URx, LLy);
            Point tl = new Point(LLx, URy);
            Point tr = new Point(URx, URy);
         
            // Translate points
            switch(startingCorner) {
            
            case LOWER_LEFT:
                bottomLeft = bl;
                bottomRight = br;
                topLeft = tl;
                topRight = tr;
                break;
                
            case LOWER_RIGHT:
                bottomLeft = br;
                bottomRight = tr;
                topLeft = bl;
                topRight = tl;
                break;
                
            case UPPER_RIGHT:
                bottomLeft = tr;
                bottomRight = tl;
                topLeft = br;
                topRight = bl;
                break;
                
            case UPPER_LEFT:
                bottomLeft = tl;
                bottomRight = bl;
                topLeft = tr;
                topRight = br;
                break;
            }
            
        }
        
        public Point getBottomLeft() {
            return bottomLeft;
        }
        
        public Point getTopRight() {
            return topRight;
        }
        
        public Point getBottomRight() {
            return bottomRight;
        }
        
        public Point getTopLeft() {
            return topLeft;
        }
        
    }
    
    /**
     * Defines the starting corner for the vehicle
     */
    public enum StartingCorner {
        LOWER_LEFT,
        LOWER_RIGHT,
        UPPER_RIGHT,
        UPPER_LEFT
    }
    
    
}
