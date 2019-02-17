package ca.mcgill.ecse211.localization;

/**
 * Orients the vehicle to a known point in global space
 */
public interface Localizer {

    /**
     * Performs a localization operation to allow the vehicle to orient itself in global space 
     */
    public void localize();
    
}
