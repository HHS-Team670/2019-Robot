/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.dataCollection.sensors;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Encoder;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Add your docs here.
 */
public class MustangEncoder {
    private Encoder DIOEncoder;
    private CANEncoder sparkEncoder;
    private boolean isDIOEncoder;

    public MustangEncoder(Encoder DIOEncoder, CANEncoder sparkEncoder) {
        this.DIOEncoder = DIOEncoder;
        this.sparkEncoder = sparkEncoder;

        isDIOEncoder = (DIOEncoder == null);
    }

    public int getPosition() {
        if (isDIOEncoder) {
            return DIOEncoder.get();
        }
        return (int)(sparkEncoder.getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION);
    }

    public double getVelocityInches(){
        if (isDIOEncoder) {
            return DIOEncoder.getRate();
        }
        return (MathUtils.convertDriveBaseTicksToInches(sparkEncoder.getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION) / 60);
    }

    public double getVelocityTicks(){
        if (isDIOEncoder) {
            return MathUtils.convertInchesToDriveBaseTicks(DIOEncoder.getRate());
        }
        return (sparkEncoder.getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
    }
}
