package ca.mcgill.ecse211.main;

import java.util.ArrayDeque;
import java.util.Queue;

import ca.mcgill.ecse211.main.FieldSearch.StartingCorner;
import ca.mcgill.ecse211.util.Board;
import lejos.robotics.geometry.Point;

public class SearchArea {
                
    private Point bottomLeft, bottomRight, topLeft, topRight;
    
    private Queue<Point> centerWaypoints;
    
    public SearchArea(int LLx, int LLy, int URx, int URy) {
                    
        
        bottomLeft = new Point(LLx, LLy);
        bottomRight = new Point(URx, LLy);
        topLeft = new Point(LLx, URy);
        topRight = new Point(URx, URy);
                    
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
        
        public Point popWaypoint() {
            return centerWaypoints.poll();
        }
        
        public Point peekNextWaypoint() {
        		return centerWaypoints.peek();
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