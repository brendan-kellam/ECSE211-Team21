package ca.mcgill.ecse211.util;

import java.awt.geom.Point2D;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.util.Log.Sender;

public final class Board {
    
    /** 
     * Represents the physical board with grid lines
     * 
     * 
     * Coordinate system:
     * y
     * ^
     * |
     *  -----> x
     *  
     *  Example <b>3x3</b> board:
     *  ________
     * |__|__|__|
     * |__|__|__|
     * |__|__|__| 
     * 
     *
     **/
   
    /**
     * Describes the current heading of the vehicle. The heading of the vehicle is entirely dependent on it's dead reckoning system (I.E. this heading describes
     * the current value of {@link ca.mcgill.ecse211.odometer.OdometerData #getTheta()}).
     * 
     */
    public enum Heading {
        N, 
        S,
        E,
        W
    }
    
    /**
     * Describes the target tunnel orientation, either horizontal or vertical, relative to starting corner 0. </br>
     * Note: Because we don't care about the competetor's tunnel, this enumeration shall only be used to describe our target tunnel.
     * 
     * </br>
     * <ul>
     * <li> Vertical:                               </li>
     * <li> *------*                                </li>
     * <li> |******|                                </li>
     * <li> |******|                                </li>
     * <li> |******|                                </li>
     * <li> *------*                                </li>
     * <li> Horizontal:                             </li>
     * <li> * -- -- -- *                            </li>
     * <li> | ******** |                            </li>
     * <li> | ******** |                            </li>
     * <li> * -- -- -- *                            </li>
     * </ul>
     * 
     */
    public enum TUNNEL_ORIENTATION {
        HORIZONTAL,
        VERTICAL
    }
    
    /**
     * Orientation of the tunnel in space. Defaults to HORIZONTAL. <br>
     * 
     * To modify, use: {@link ca.mcgill.ecse211.util.Board #setTunnelOrientation()}
     */
    private static TUNNEL_ORIENTATION curTunnelOrientation = TUNNEL_ORIENTATION.HORIZONTAL;
    
    
    // Minimum tile multiple
    private static final double MIN_TILE_MULTIPLE = 0.5;
    
    // Tile size
    public static final double TILE_SIZE = 30.48;
    
    /**
     * Board width. 
     * 
     * @see <a href="https://mycourses2.mcgill.ca/d2l/le/content/372671/viewContent/4289469/View">Project Spec</a>
     */
    public static final int BOARD_WIDTH = 15;
    
    /**
     * Board height. 
     * 
     * @see <a href="https://mycourses2.mcgill.ca/d2l/le/content/372671/viewContent/4289469/View">Project Spec</a>
     */
    public static final int BOARD_HEIGHT = 9;
    
    
    /**
     * Determine vehicle's heading </br>
     * 
     * <b> NOTE: </b> This method will bound theta to: 0 <= theta < 360 <br>
     * @see EV3Math.boundAngle(theta)
     * 
     * @param theta
     * @return Heading
     */
    public static Heading getHeading(double theta) {
        
        theta = EV3Math.boundAngle(theta);
                        
        if (theta <= 45) {
            return Heading.N;
        } else if (theta <= 135) {
            return Heading.E;
        } else if (theta <= 225) {
            return Heading.S;
        } else if (theta <= 315) {
            return Heading.W;
        } else {
            return Heading.N;
        }
    }
    
    /**
     * Convert a heading to a angle
     * 
     * @param Robot heading
     * @return angle representing the heading
     */
    public static double getHeadingAngle(Heading heading) {
        switch (heading) {
        case N:
            return 0;
        case E:
            return 90;
        case S:
            return 180;
        case W:
            return 270;
        }
        
        return 0;
    }
    
    /**
     * Starting corner translation
     */
    public static final Point2D scTranslation[] = {
            new Point2D.Double(Board.TILE_SIZE, Board.TILE_SIZE),
            new Point2D.Double(Board.TILE_SIZE * (Board.BOARD_WIDTH - 1), Board.TILE_SIZE),
            new Point2D.Double(Board.TILE_SIZE * (Board.BOARD_WIDTH - 1), Board.TILE_SIZE * (Board.BOARD_HEIGHT - 1)),
            new Point2D.Double(Board.TILE_SIZE, Board.TILE_SIZE * (Board.BOARD_HEIGHT - 1)),
    };
    
    
    public static final double scRotation[] = {
            0.0,
            270.0,
            180.0,
            90.0
    };
    
    /**
     * Return true if headings are orthogonal
     * 
     * @param h1
     * @param h2
     */
    public static boolean areHeadingsOrthogonal(Heading h1, Heading h2) {
        return EV3Math.distance(getHeadingAngle(h1), getHeadingAngle(h2)) == 90.0;
    }
    
    /**
     * Return true if headings are parallel
     * 
     * @param h1
     * @param h2
     * 
     */
    public static boolean areHeadingsParallel(Heading h1, Heading h2) {
        return !areHeadingsOrthogonal(h1, h2);
    }
    
    public static Heading getOrthogonalHeading(Heading h) {
        return getHeading(getHeadingAngle(h) - 90.0);
    }
    
    
    private enum Quadrant
    {
        I,
        II,
        III,
        IV
    }
    
    public static Heading getOrthogonalHeading(Heading h, Tile tile) {
        
        double x = tile.getCenter().getX(), y = tile.getCenter().getY();
        
        double midx = (BOARD_WIDTH * TILE_SIZE)/2;
        double midy = (BOARD_HEIGHT * TILE_SIZE)/2;
        
        double angle = getHeadingAngle(h);
        Heading fin = h;
        
        switch (h) {
            case N:
            {
                if (x > midx) {
                    fin = getHeading(angle - 90.0);
                } else {
                    fin = getHeading(angle + 90.0);
                }
                break;
            }
        
            case S:
            {
                if (x > midx) {
                    fin = getHeading(angle + 90.0);
                } else {
                    fin = getHeading(angle - 90.0);
                }
                break;
            }
            
            case E:
            {
                if (y > midy) {
                    fin = getHeading(angle + 90.0);
                } else {
                    fin = getHeading(angle - 90.0);
                }
                break;
            }
        
            case W:
            {
                if (y > midy) {
                    fin = getHeading(angle - 90.0);
                } else {
                    fin = getHeading(angle + 90.0);
                }
                break;
            }
        
        }
        
        return fin;
    }
    
    public static Heading getParallelHeading(Heading h) {
        return getHeading(getHeadingAngle(h) + 180.0);
    }
    
    /**
     * Snaps to a given gridline.
     * 
     * Given a position, the method will determine the heading of the robot (either the x or y component) and determine
     * the closest gridline to it's current position in said heading
     * 
     * @param position
     * @param odometer
     */
    public static void snapToGridLine(Odometer odometer, boolean withLightSensorOffset) {
        
        // Get x, y, theta
        double x = Odometer.getX();
        double y = Odometer.getY();
        double theta = Odometer.getTheta();
        
        Log.log(Sender.board, "snapToGridLine:: Odometer X and Y: (" + Odometer.getX() + ", " + Odometer.getY() + ")");
        
        double xoffset = 0.0, yoffset = 0.0;
        
        if (withLightSensorOffset) {
            double offset[] = determineLightOffset();
            xoffset = offset[0];
            yoffset = offset[1];
        }   

        // Apply the offset
        x += xoffset;
        y += yoffset;
        
        Log.log(Sender.board, "snapToGridLine:: Offset X and Y: (" + x + ", " + y + ")");
        
        // Fills the new position of the robot after correction
        double pos; 
        
        Heading heading = getHeading(theta);
       
        // Moving in the y direction
        if (heading == Heading.N || heading == Heading.S) {
            pos = y;
        }
        
        // Moving in the x direction
        else {
            pos = x;
        }
        
        // For the current position, determine the number of tileLength multiples
        double div = pos / TILE_SIZE;
        
//        // If the multiple is less than the minimum multiple
//        if (Math.abs(div) < MIN_TILE_MULTIPLE) {
//            // Reset position to 0
//            pos = 0;
//            
//        // Otherwise, set position to the rounded multiple multiplied by the tile length
//        } else {
            pos = TILE_SIZE * Math.round(div);
//        }
        
        String msg = "snapToGridLine:: Snap to ";
                
        // Moving in the y direction
        if (heading == Heading.N || heading == Heading.S) {
            
            // Remove the yoffset from the position
            pos -= yoffset;
            odometer.setY(pos);
            msg += "Y: ";
        }
        // Moving in the x direction
        else {
            
            // Remove the xoffset from the position
            pos -= xoffset; 
            odometer.setX(pos);
            msg += "X: ";
        }       
        
        msg += pos;
        
        Log.log(Log.Sender.board, msg);
    }
    
    
    /**
     * When a line is detected, and assuming the vehicle is orthogonal to the line, the vehicle is pointed towards either N, E, S or W. 
     * Depending on this direction, the position of the light sensors will be differently offset from the origin of the robot.
     * <b>The origin</b> of the robot is located in the center of the wheel base. The following are the offsets:
     * 
     * <ol>
     * <li>E: x - 13cm</li>
     * <li>W: x + 13cm</li>
     * <li>N: y - 13cm</li>
     * <li>S: y + 13cm</li>
     * </ol>     
     * 
     * This function returns a double array with two elements: xOffset and yOffset. <b>NOTE:</b> one of these components will always be 0.
     * The other will either be ±{@link ca.mcgill.ecse211.util.Vehicle #VERT_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE DISTANCE_FROM_LIGHT_SENSORS_TO_WHEEL_BASE}
     * 
     */
    public static double[] determineLightOffset() {
        
        Heading heading = getHeading(Odometer.getTheta());
        
        if (heading == Heading.E) {
            return new double[] {-Vehicle.VERT_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE, 0.0};
        }
        else if (heading == Heading.W) {
            return new double[] {Vehicle.VERT_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE, 0.0};
        }
        else if (heading == Heading.N) {
            return new double[] {0.0, -Vehicle.VERT_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE};
        } else {
            return new double[] {0.0, Vehicle.VERT_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE};
        }
        
    }
    
    public static void snapToHeading(Odometer odometer) {
        Heading heading = getHeading(Odometer.getTheta());
        odometer.setTheta(getHeadingAngle(heading));        
    }
    
    /**
     * Sets the tunnel's {@link ca.mcgill.ecse211.util.Board #curTunnelOrientation orientation} to the correct 
     * {@link ca.mcgill.ecse211.util.Board.TUNNEL_ORIENTATION TUNNEL_ORIENTATION}. This orientation is computed by comparing the length
     * and height of the tunnel to see which is larger. (length > height ==> HORIZONTAL), (height > length ==> VERTICLE)
     * 
     * @param llx - lower left x of tunnel
     * @param lly - lower left y of tunnel
     * @param urx - upper right x of tunnel
     * @param ury - upper right y of tunnel
     * 
     * @throws IllegalArgumentException If the tunnel length or height is invalid (I.E doesn't form a rectangle that's 2x1 or 1x2)
     */
    public static void setTunnelOrientation(int llx, int lly, int urx, int ury) throws IllegalArgumentException {
        
        double length = Math.abs(llx - urx);
        double height = Math.abs(lly - ury);
        
        // Sanity check
        if ((length >= 2 && height >= 2) || (length < 0 || height < 0) || (Math.abs(length - height) != 1)) {
            throw new IllegalArgumentException("The tunnel is a rectangle with dimension 2x1");
        }
        
        if (length > height) {
            Board.curTunnelOrientation = TUNNEL_ORIENTATION.HORIZONTAL;
        } else {
            Board.curTunnelOrientation = TUNNEL_ORIENTATION.VERTICAL;
        }
    }
    
    public static TUNNEL_ORIENTATION getTunnelOrientation() {
        return Board.curTunnelOrientation;
    }
       
    
}