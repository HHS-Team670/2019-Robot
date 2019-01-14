/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.dataCollection;

import com.ctre.phoenix.motorcontrol.SensorCollection;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class SensorCollection_PIDSource implements PIDSource {
    
    private SensorCollection collection;
    private PIDSourceType pidSourceType; 

    public SensorCollection_PIDSource(SensorCollection collection) {
        this.collection = collection;
        pidSourceType = PIDSourceType.kDisplacement;
    }
    
    @Override
    public PIDSourceType getPIDSourceType() {
        return pidSourceType;
    }

    @Override
    public double pidGet() {
        return collection.getQuadraturePosition();
    }


    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        this.pidSourceType = pidSource;
    }
}