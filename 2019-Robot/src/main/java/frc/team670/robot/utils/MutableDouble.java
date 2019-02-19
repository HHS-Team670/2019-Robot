/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils;

public class MutableDouble {

    private double value;
 
    public MutableDouble(double value) {
      this.value = value;
    }
 
    public double getValue() {
      return this.value;
    }
 
    public void setValue(double value) {
      this.value = value;
    }
  }