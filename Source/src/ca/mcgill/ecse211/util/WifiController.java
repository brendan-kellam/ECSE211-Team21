package ca.mcgill.ecse211.util;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;

public class WifiController {

    // ** Set these as appropriate for your team and current situation **//
    private static final String SERVER_IP = "192.168.2.27";
    private static final int TEAM_NUMBER = 21;

    // Enable/disable printing of debug info from the WiFi class
    private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
    
    private enum TUNNEL_ORIENTATION {
        HORIZONTAL,
        VERTICAL
    }
    
    public static void fetchGameplayData() {
        // Initialize WifiConnection class
        WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
        
        try {
            Map data = conn.getData();
            
            Board.Config.redTeam = getInt(data, "RedTeam");
            Board.Config.greenTeam = getInt(data, "GreenTeam");           
            
            Board.Config.islandLL = Tile.lowerLeft(getInt(data, "Island_LL_x"),  getInt(data, "Island_LL_y")); 
            Board.Config.islandUR = Tile.upperRight(getInt(data, "Island_UR_x"), getInt(data, "Island_UR_y"));
            
            // Tunnel params
            int tnLLX = 0, tnLLY = 0, tnURX = 0, tnURY = 0;
            
            // Red team
            if (Board.Config.redTeam == TEAM_NUMBER) {
                Board.Config.corner = getInt(data, "RedCorner");
                
                Board.Config.startingAreaLL = Tile.lowerLeft(getInt(data,  "Red_LL_x"),    getInt(data, "Red_LL_y"));
                Board.Config.startingAreaUR = Tile.upperRight(getInt(data, "Red_UR_x"),    getInt(data, "Red_UR_y"));            

                Board.Config.searchAreaLL = Tile.lowerLeft(getInt(data, "SZR_LL_x"), getInt(data, "SZR_LL_y"));
                Board.Config.searchAreaUR = Tile.upperRight(getInt(data, "SZR_UR_x"), getInt(data, "SZR_UR_y"));
                
                tnLLX = getInt(data, "TNR_LL_x");
                tnLLY = getInt(data, "TNR_LL_y");
                tnURX = getInt(data, "TNR_UR_x"); 
                tnURY = getInt(data, "TNR_UR_y");
            }
            
            // Green team
            else if (Board.Config.greenTeam == TEAM_NUMBER) {
                Board.Config.corner = getInt(data, "GreenCorner");
                
                Board.Config.startingAreaLL = Tile.lowerLeft(getInt(data, "Green_LL_x"),     getInt(data, "Green_LL_y"));
                Board.Config.startingAreaUR = Tile.upperRight(getInt(data, "Green_UR_x"),    getInt(data, "Green_UR_y"));            
                
                Board.Config.searchAreaLL = Tile.lowerLeft(getInt(data, "SZG_LL_x"), getInt(data, "SZG_LL_y"));
                Board.Config.searchAreaUR = Tile.upperRight(getInt(data, "SZG_UR_x"), getInt(data, "SZG_UR_y"));
                
                tnLLX = getInt(data, "TNG_LL_x");
                tnLLY = getInt(data, "TNG_LL_y");
                tnURX = getInt(data, "TNG_UR_x");
                tnURY = getInt(data, "TNG_UR_y");
                
            } else {
                throw new RuntimeException("Neither green team number nor red team number match TEAM_NUMBER: " + TEAM_NUMBER);
            }
            
            // Horizontal
            if (getOrientation(tnLLX, tnLLY, tnURX, tnURY) == TUNNEL_ORIENTATION.HORIZONTAL) {
                Board.Config.tunnelLL = Tile.lowerRight(tnLLX, tnLLY);
                Board.Config.tunnelUR = Tile.upperLeft(tnURX, tnURY);
            
            // Vertical
            } else {
                Board.Config.tunnelLL = Tile.upperLeft(tnLLX, tnLLY);
                Board.Config.tunnelUR = Tile.lowerRight(tnURX, tnURY);
            }
            
        } catch (Exception e) {
            System.err.println("Error: " + e.getMessage());
        }
        
    }
    
    private static int getInt(Map data, String key) {
        return ((Long) data.get(key)).intValue();
    }
    
    private static TUNNEL_ORIENTATION getOrientation(int llx, int lly, int urx, int ury) {
        if (Math.abs(llx - urx) > Math.abs(lly - ury)) {
            return TUNNEL_ORIENTATION.HORIZONTAL;
        } else {
            return TUNNEL_ORIENTATION.VERTICAL;
        }
    }
}
