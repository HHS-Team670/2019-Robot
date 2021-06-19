/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import java.util.List;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.team670.robot.commands.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.cameras.FlipDriverCameraMode;
import frc.team670.robot.commands.claw.CloseClaw;
import frc.team670.robot.commands.claw.OpenClaw;
import frc.team670.robot.commands.drive.teleop.FlipDriveAndCamera;
import frc.team670.robot.commands.drive.teleop.ToggleDriveSafe;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.XKeys;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.utils.MustangController;
import frc.team670.robot.utils.MustangController.XboxButtons;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // Controllers/Joysticks
  private MustangController driverController;

  private XKeys xkeys;

  // Buttons
  private JoystickButton toggleReverseDrive, toggleDriverCameraMode, toggleChildSafe;
  private JoystickButton armToNeutral;  

  private JoystickButton dropBall, grabBall;

  public OI() {
    driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
    // operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);
    xkeys = new XKeys();
    toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    toggleReverseDrive.whenPressed(new FlipDriveAndCamera());
    toggleChildSafe = new JoystickButton(driverController, XboxButtons.X);
    toggleChildSafe.whenPressed(new ToggleDriveSafe());
    toggleDriverCameraMode = new JoystickButton(driverController, XboxButtons.B);
    toggleDriverCameraMode.whenPressed(new FlipDriverCameraMode());
    armToNeutral = new JoystickButton(driverController, XboxButtons.A);
    armToNeutral.whenPressed(
      new InstantCommand() {
        protected void initialize() {
          Scheduler.getInstance().add(new MoveArm(Arm.getArmState(LegalState.NEUTRAL), Robot.arm));
        }
      }
    );

    // FOR TESTING OPENING/CLOSING the claw
    // dropBall = new JoystickButton(driverController, XboxButtons.Y);
    // dropBall.whenPressed(new OpenClaw(Robot.claw));
    // grabBall = new JoystickButton(driverController, XboxButtons.X);
    // grabBall.whenPressed(new CloseClaw(Robot.claw));

    // dropBall = new JoystickButton(driverController, XboxButtons.B);
    // dropBall.whenPressed(new InstantCommand() {
    //   protected void initialize() {
    //     Robot.claw.togglePush();
    //   }
    // });

  }
  /**
   * Sets the rumble on the driver controller
   * 
   * @param power The desired power of the rumble [0, 1]
   * @param time The time to rumble for in seconds
   */
  public void rumbleDriverController(double power, double time) {
    rumbleController(driverController, power, time);
  }

  /**
   * Sets the rumble on the operator controller
   * 
   * @param power The desired power of the rumble [0, 1]
   * @param time The time to rumble for in seconds
   */
  public void rumbleOperatorController(double power, double time) {
    // rumbleController(operatorController, power, time);
  }

  private void rumbleController(MustangController controller, double power, double time) {
    controller.rumble(power, time);
  }

  public MustangController getDriverController() {
    return driverController;
  }

  // public MustangController getOperatorController() {
  //   return operatorController;
  // }

  public boolean isQuickTurnPressed() {
    return driverController.getRightBumper();
  }

  //CHEESE
    //https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/creating-following-trajectory.html
  public SequentialCommandGroup getAutonCommand2021(DriveBase driveBase){
    
    //TODO: FIND CONSTANTS IN EXPERIMENATTION BRANCH
    double leftKsVolts = 0.246;
    double kvVoltSecondsPerMeter = 2.1;
    double leftKaVoltSecondsSquaredPerMeter = 0.2;
    double kTrackwidthMeters = 0.702;
    double kMaxSpeedMetersPerSecond = 1.2;
    double kMaxAccelerationMetersPerSecondSquared = 1.22;
    double kRamseteB = 2;
    double kRamseteZeta = .7;
    double leftKPDriveVel = 6;
    double rightKPDriveVel = 2.4;
    double rightKsVolts = 0.12;

    DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
      kTrackwidthMeters);
    // Create a voltage constraint to ensure we don't accelerate too fast
    //set to left values
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(leftKsVolts,
                                       kvVoltSecondsPerMeter,
                                       leftKaVoltSecondsSquaredPerMeter), kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

     RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        driveBase::getPose,
        new RamseteController(kRamseteB, kRamseteZeta),
        new SimpleMotorFeedforward(leftKsVolts,
                                   kvVoltSecondsPerMeter,
                                   leftKaVoltSecondsSquaredPerMeter),
        kDriveKinematics,
        driveBase::getWheelSpeeds,
        new PIDController(leftKPDriveVel, 0, 0),
        new PIDController(rightKPDriveVel 0, 0),
        // RamseteCommand passes volts to the callback
        driveBase::tankDriveVolts,
        driveBase
    );

    // Reset odometry to the starting pose of the trajectory.
    driveBase.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveBase.tankDriveVoltage(0, 0));
    
  }

  //CHEESE
  //https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/creating-following-trajectory.html
  public Command getSelectedAutonCommand() {

    SmartDashboard.putBoolean("auto-command", xkeys.getAutonCommand() == null);
    return xkeys.getAutonCommand();
  }
}
