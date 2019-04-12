package ca.mcgill.ecse211.claw;

import ca.mcgill.ecse211.sensor.UltrasonicPoller;
import ca.mcgill.ecse211.util.Vehicle;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class Claw {

    private enum ClawState {
        OPEN,
        GRABBED,
        STOWED
    }
    
    
    // Claw starts out open
    private ClawState currentState = ClawState.OPEN;
    
	/**
	 * Colour sensor motor
	 */
	//One motor for the light sensor.
	public final static EV3MediumRegulatedMotor clawMotor = Vehicle.CLAW_MOTOR;
	// Ultrasonic poller
	private UltrasonicPoller usPoller;
	
	public Claw(UltrasonicPoller usPoller) {
 		this.usPoller = usPoller;
	}
	
	/**
	 * Allow the claw to grab a can that is directly in front of the robot.
	 */
	public void grab() {
	    if (currentState == ClawState.OPEN) {
		    clawMotor.rotate(-130);
		    currentState = ClawState.GRABBED;
	    }
	}
	
	/**
	 * Stow the claw in order to enter the tunnel cleanly
	 */
	public void stow() {
	    if (currentState == ClawState.OPEN) {
            clawMotor.rotate(-50);
            currentState = ClawState.STOWED;
        }
	}
	
	/**
	 * Allow the claw to release the can that it is holding 
	 */
	public void release() {
	    
	    // Can't release if claw is already open
	    if (currentState == ClawState.OPEN) {
	        return;
	    }
	    
	    if (currentState == ClawState.GRABBED) {
		    clawMotor.rotate(120);
	    } else {
	        clawMotor.rotate(50);
	    }
	    
        currentState = ClawState.OPEN;
	}
	
	/*
	 * Allow the motor to approach the can at a distance that can grab it.
	 */
	public void approachCan() {
		Vehicle.setMotorSpeeds(100, 100); 
		while (usPoller.getDistance() > 3);
		Vehicle.setMotorSpeeds(0, 0);
	}
	
	
}
