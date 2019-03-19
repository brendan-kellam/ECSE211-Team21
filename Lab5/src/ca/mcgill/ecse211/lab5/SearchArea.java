package ca.mcgill.ecse211.lab5;

import java.util.ArrayDeque;
import java.util.Queue;

import ca.mcgill.ecse211.lab5.FieldSearch.StartingCorner;
import ca.mcgill.ecse211.util.Board;
import lejos.robotics.geometry.Point;

public class SearchArea {
                
    private Point bottomLeft, bottomRight, topLeft, topRight;
    private StartingCorner startingCorner;
    
    private Queue<Point> centerWaypoints;
    
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
                    
        this.centerWaypoints = new ArrayDeque<>();
        
        float xCur;
        float yCur;
        double xOff = getBottomLeft().getX() * Board.TILE_SIZE;
        double yOff = getBottomLeft().getY() * Board.TILE_SIZE;
        
        for (int x = 0; x < (int) getWidth(); x++) {
            
            if (x % 2 == 0) {
                for (int y = 0; y < (int) getHeight(); y++) {
                    
                    xCur = (float) (x * Board.TILE_SIZE + xOff + Board.TILE_SIZE / 2);
                    yCur = (float) (y * Board.TILE_SIZE + yOff + Board.TILE_SIZE / 2);
                    
                    centerWaypoints.add(new Point(xCur, yCur));
                    
                }
            } else {
                for (int y = (int) getHeight()-1; y>= 0; y--) {
                    
                    xCur = (float) (x * Board.TILE_SIZE + xOff + Board.TILE_SIZE / 2);
                    yCur = (float) (y * Board.TILE_SIZE + yOff + Board.TILE_SIZE / 2);
                    
                    centerWaypoints.add(new Point(xCur, yCur));
                    }
                }
                
            }
            
        }
        
        public Point getNextWaypoint() {
            return centerWaypoints.poll();
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
        
        public double getHeight() {
            return this.topLeft.getY() - this.bottomLeft.getY();
        }
        
        public double getWidth() {
            return this.bottomRight.getX() - this.bottomLeft.getX();
        }

    }
    

