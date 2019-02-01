/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.dataCollection.sensors;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * A Mustang Encoder holds both a DIOEncoder and a Spark Encoder. The Spark Encoder will only be used if
 * the DIOEncoder is not plugged in properly (is null).
 */
public class MustangDriveBaseEncoder implements PIDSource {
    private PIDSourceType type;
    private Encoder dioEncoder;
    private CANEncoder sparkEncoder;
    private boolean isDIOEncoder;

    /**
     * @param dioEncoder the DIO Encoder used for this side of the drivebase
     * @param sparkEncoder the Spark Encoder associatd for this side of the drivebase
     */
    public MustangDriveBaseEncoder(Encoder dioEncoder, CANEncoder sparkEncoder) {
        this.dioEncoder = dioEncoder;
        this.sparkEncoder = sparkEncoder;
        type = PIDSourceType.kDisplacement;
        isDIOEncoder = (dioEncoder == null);
    }

    /**
     * Returns the position determined by the encoder in ticks
     */
    public int getPositionTicks() {
        if (isDIOEncoder) {
            return dioEncoder.get();
        }
        return (int) (sparkEncoder.getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION);
    }

    /**
     * Returns the position determined by the encoder in inches
     */
    public double getPositionInches() {
        if (isDIOEncoder) {
            return MathUtils.convertDriveBaseTicksToInches(dioEncoder.get());
        }
        return MathUtils.convertDriveBaseSparkTicksToInches((sparkEncoder.getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION));
    }


    /**
     * Returns the velocity determined by the encoder in inches/second
     */
    public double getVelocityInches() {
        if (isDIOEncoder) {
            return dioEncoder.getRate();
        }
        return (MathUtils.convertDriveBaseTicksToInches(sparkEncoder.getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION) / 60);
    }

     /**
     * Returns the velocity determined by the encoder in ticks/second
     */
    public double getVelocityTicks() {
        if (isDIOEncoder) {
            return MathUtils.convertInchesToDriveBaseTicks(dioEncoder.getRate());
        }
        return (sparkEncoder.getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        type = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return type;
    }

    @Override
    public double pidGet() {
        return getPositionTicks();
    }
}
