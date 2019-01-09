/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import frc.team957.LEDControl;

/**
 * Code for communicating LED strip changes to the RoboRIO. You will need to modify this to interface with its superclass
 * (LEDControl) so that it has an enum
 */
public class MustangLEDs extends LEDControl {

    /**
     * Represents the different patterns that can be displayed by the strip.
     */
    public enum LEDState{NEUTRAL};

    private LEDState state;

    /**
     * Instantiates MustangLEDs for an LED strip with length 'length'
     */
    MustangLEDs(int length) {
        super(length);
        state = LEDState.NEUTRAL;
    }

    /**
     * Returns the current state of the LED strip (the pattern currently being displayed)
     */
    public LEDState getCurrentState() {
        return state;
    }

    public void setLEDState(LEDState state) {
        this.state = state;
    }

}
