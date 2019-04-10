package ca.mcgill.ecse211.util;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.main.CompetitionConfig;
import ca.mcgill.ecse211.util.Board.Heading;
import lejos.hardware.Sound;

public class WifiController {

    // ** Set these as appropriate for your team and current situation **//
    private static final String SERVER_IP = "192.168.2.21";
    private static final int TEAM_NUMBER = 21;

    // Enable/disable printing of debug info from the WiFi class
    private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
    
   
    
    public static void fetchGameplayData() {
        // Initialize WifiConnection class
        WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
        
        try {
            Map data = conn.getData();
            
            CompetitionConfig.redTeam = getInt(data, "RedTeam");
            CompetitionConfig.greenTeam = getInt(data, "GreenTeam");           
            
            CompetitionConfig.islandLL = Tile.lowerLeft(getInt(data, "Island_LL_x"),  getInt(data, "Island_LL_y")); 
            CompetitionConfig.islandUR = Tile.upperRight(getInt(data, "Island_UR_x"), getInt(data, "Island_UR_y"));
            
            // Tunnel params
            int tnLLX = 0, tnLLY = 0, tnURX = 0, tnURY = 0;
            
            // Red team
            if (CompetitionConfig.redTeam == TEAM_NUMBER) {
                CompetitionConfig.corner = getInt(data, "RedCorner");
                
                CompetitionConfig.startingAreaLL = Tile.lowerLeft(getInt(data,  "Red_LL_x"),    getInt(data, "Red_LL_y"));
                CompetitionConfig.startingAreaUR = Tile.upperRight(getInt(data, "Red_UR_x"),    getInt(data, "Red_UR_y"));            

                CompetitionConfig.searchAreaLL = Tile.lowerLeft(getInt(data, "SZR_LL_x"), getInt(data, "SZR_LL_y"));
                CompetitionConfig.searchAreaUR = Tile.upperRight(getInt(data, "SZR_UR_x"), getInt(data, "SZR_UR_y"));
                
                tnLLX = getInt(data, "TNR_LL_x");
                tnLLY = getInt(data, "TNR_LL_y");
                tnURX = getInt(data, "TNR_UR_x"); 
                tnURY = getInt(data, "TNR_UR_y");
            }
            
            // Green team
            else if (CompetitionConfig.greenTeam == TEAM_NUMBER) {
                CompetitionConfig.corner = getInt(data, "GreenCorner");
                
                CompetitionConfig.startingAreaLL = Tile.lowerLeft(getInt(data, "Green_LL_x"),     getInt(data, "Green_LL_y"));
                CompetitionConfig.startingAreaUR = Tile.upperRight(getInt(data, "Green_UR_x"),    getInt(data, "Green_UR_y"));            
                
                CompetitionConfig.searchAreaLL = Tile.lowerLeft(getInt(data, "SZG_LL_x"), getInt(data, "SZG_LL_y"));
                CompetitionConfig.searchAreaUR = Tile.upperRight(getInt(data, "SZG_UR_x"), getInt(data, "SZG_UR_y"));
                
                tnLLX = getInt(data, "TNG_LL_x");
                tnLLY = getInt(data, "TNG_LL_y");
                tnURX = getInt(data, "TNG_UR_x");
                tnURY = getInt(data, "TNG_UR_y");
                
            } else {
                throw new RuntimeException("Neither green team number nor red team number match TEAM_NUMBER: " + TEAM_NUMBER);
            }
            
            // Set the tunnel orientation
            try {
                Board.setTunnelOrientation(tnLLX, tnLLY, tnURX, tnURY);
            } catch (IllegalArgumentException e) {
                throw e;
            }
                
            configureTunnelEntryPoints(tnLLX, tnLLY, tnURX, tnURY);
            
            
        } catch (Exception e) {
            System.err.println("Error: " + e.getMessage());
        }
    }
    
    /**
     * Configures the tunnel entry tiles.
     * 
     * @param tnURY 
     * @param tnURX 
     * @param tnLLY 
     * @param tnLLX 
     */
    private static void configureTunnelEntryPoints(int tnLLX, int tnLLY, int tnURX, int tnURY) {
        
        
        // Configure if
        if (Board.getTunnelOrientation() == Board.TUNNEL_ORIENTATION.HORIZONTAL) {
            
            // We have the assurance that leftTile.getMaxX() < rightTile.getMinX()
            Tile leftTile = Tile.lowerRight(tnLLX, tnLLY);
            Tile rightTile = Tile.upperLeft(tnURX, tnURY);
            
            double startX = Board.scTranslation[CompetitionConfig.corner].getX();
            
            // We must travel E to reach the search area
            if (startX < leftTile.getMaxX()) {
                CompetitionConfig.tunnelEntranceToSearchArea = leftTile;
                CompetitionConfig.tunnelEntranceToStartArea = rightTile;
                CompetitionConfig.toSearchAreaHeading = Heading.E;
                CompetitionConfig.toStartAreaHeading = Heading.W;
          
            // Else, we must travel W to reach the search area
            } else {
                CompetitionConfig.tunnelEntranceToSearchArea = rightTile;
                CompetitionConfig.tunnelEntranceToStartArea = leftTile;
                CompetitionConfig.toSearchAreaHeading = Heading.W;
                CompetitionConfig.toStartAreaHeading = Heading.E;
            }

            
        // Vertical
        } else {
            
            Tile bottomTile = Tile.upperLeft(tnLLX, tnLLY);
            Tile topTile = Tile.lowerRight(tnURX, tnURY);
            
            double startY = Board.scTranslation[CompetitionConfig.corner].getY();

            // We must travel N to reach the search area
            if (startY < bottomTile.getMaxY()) {
                CompetitionConfig.tunnelEntranceToSearchArea = bottomTile;
                CompetitionConfig.tunnelEntranceToStartArea = topTile;
                CompetitionConfig.toSearchAreaHeading = Heading.N;
                CompetitionConfig.toStartAreaHeading = Heading.S;
            }

            // Else, we must travel S to reach the search area
            else {
                CompetitionConfig.tunnelEntranceToSearchArea = topTile;
                CompetitionConfig.tunnelEntranceToStartArea = bottomTile;
                CompetitionConfig.toSearchAreaHeading = Heading.S;
                CompetitionConfig.toStartAreaHeading = Heading.N;
            }
            
        }
    }
    
    private static int getInt(Map data, String key) {
        return ((Long) data.get(key)).intValue();
    }
    
    
}
