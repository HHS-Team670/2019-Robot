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
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * A Mustang Encoder holds both a DIOEncoder and a Spark Encoder. The Spark Encoder will only be used if
 * the DIOEncoder is not plugged in properly (is null). Does PIDOutput in ticks.
 */
public class MustangDriveBaseEncoder implements PIDSource {
    private PIDSourceType type;
    private Encoder dioEncoder;
    private CANEncoder sparkEncoder;
    private boolean isDIOEncoder;
    private boolean isSparkMAXInverted;

    /**
     * @param dioEncoder the DIO Encoder used for this side of the drivebase
     * @param sparkEncoder the Spark Encoder associatd for this side of the drivebase
     */
    public MustangDriveBaseEncoder(Encoder dioEncoder, CANEncoder sparkEncoder, boolean isSparkMAXInverted) {
        this.dioEncoder = dioEncoder;
        this.sparkEncoder = sparkEncoder;
        type = PIDSourceType.kDisplacement;
        this.isSparkMAXInverted = isSparkMAXInverted;
        isDIOEncoder = (dioEncoder != null);
    }

    /**
     * Returns the position determined by the encoder in ticks
     */
    public int getPositionTicks() {
        try {
            if (isDIOEncoder) {
                return dioEncoder.get();
            }
            return (isSparkMAXInverted ? -1 : 1) * (int) (sparkEncoder.getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION);
        }  
        catch(RuntimeException e) {
            return 0;
        }
    }

    /**
     * Returns the position determined by the encoder in inches
     */
    public double getPositionInches() {
        try {
            if (isDIOEncoder) {
                return DriveBase.convertDriveBaseTicksToInches(dioEncoder.get());
            }
            return (isSparkMAXInverted ? -1 : 1) * DriveBase.convertSparkRevolutionsToInches(sparkEncoder.getPosition());
        }
        catch(RuntimeException e) {
            return 0;
        }
    }


    /**
     * Returns the velocity determined by the encoder in inches/second
     */
    public double getVelocityInches() {
        try {
            if (isDIOEncoder) {
                return dioEncoder.getRate();
            }
            return (isSparkMAXInverted ? -1 : 1) * (DriveBase.convertDriveBaseTicksToInches(sparkEncoder.getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION) / 60);
        }
        catch(RuntimeException e) {
            return 0;
        }
    }

     /**
     * Returns the velocity determined by the encoder in ticks/second
     */
    public double getVelocityTicks() {
        try {
            if (isDIOEncoder) {
                return DriveBase.convertInchesToDriveBaseTicks(dioEncoder.getRate());
            }
            return (isSparkMAXInverted ? -1 : 1) * (sparkEncoder.getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
        }
        catch(RuntimeException e) {
            return 0;
        }
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
