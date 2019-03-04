/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.vision;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Starts a Pure Pursuit path based off vision data
 */
public class CollectVisionData extends Command {

    private MustangCoprocessor coprocessor;

    private boolean lowTarget, isReversed;
    private double[] visionData;
    private long startTime;

    private static final double MAX_TIME_TO_RUN = 700; // Max time to run this in ms

    public CollectVisionData(double[] visionData, MustangCoprocessor coprocessor, boolean lowTarget, boolean isReversed) {
        super();
        this.visionData = visionData;
        this.lowTarget = lowTarget;
        this.isReversed = isReversed;
        this.coprocessor = coprocessor;
    }

    @Override
    protected void initialize() {
        coprocessor.turnOnBackLedRing();
        coprocessor.setTargetHeight(lowTarget);
        coprocessor.setCamera(isReversed);
        SmartDashboard.putNumberArray("reflect_tape_data", new double[]{RobotConstants.VISION_ERROR_CODE,RobotConstants.VISION_ERROR_CODE,RobotConstants.VISION_ERROR_CODE}); // Clears vision data so we don't use old data accidentally
        coprocessor.useVision(true);
        startTime = System.currentTimeMillis();
    }

    @Override
    protected void execute() {

    }

    @Override
    protected boolean isFinished() {
        return (!MathUtils.doublesEqual(visionData[2], RobotConstants.VISION_ERROR_CODE) || System.currentTimeMillis() > startTime + MAX_TIME_TO_RUN);
    }

    @Override
    protected void end() {
        coprocessor.turnOffBackLedRing();
        SmartDashboard.putString("vision-camera", "disabled");
    }

    @Override
    protected void interrupted() {
        end();
    }

}
