/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.arm.zero.SafelyResetExtension;
import frc.team670.robot.commands.drive.teleop.XboxRocketLeagueDrive;
import frc.team670.robot.commands.drive.straight.TimeDrive;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.HeldItem;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangLEDs_2019;
import frc.team670.robot.subsystems.elbow.Elbow;
import frc.team670.robot.subsystems.extension.Extension;
import frc.team670.robot.subsystems.wrist.Wrist;
import frc.team670.robot.utils.Logger;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.List;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;
  public static MustangSensors sensors = new MustangSensors();
  public static MustangCoprocessor coprocessor = new MustangCoprocessor();
  public static DriveBase driveBase = new DriveBase();
  public static MustangLEDs_2019 leds = new MustangLEDs_2019();

  private static Elbow elbow = new Elbow();
  private static Wrist wrist = new Wrist();
  private static Extension extension = new Extension();
  public static Intake intake = new Intake();
  public static Claw claw= new Claw();
  public static Arm arm = new Arm(elbow, wrist, extension, intake, claw);

  private Notifier updateArbitraryFeedForwards;

  private Command autonomousCommand, operatorControl;
  private SendableChooser<Command> auton_chooser = new SendableChooser<>();

  private static final double NETWORK_TABLES_UPDATE_RATE = 0.05;

  private boolean firstTimeEnteringTeleop;
  
  public Robot() {
  }

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // auton_chooser.addDefault("Default Auto", new TimeDrive());
    // chooser.addObject("My Auto", new MyAutoCommand());

    NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    networkTables.setUpdateRate(NETWORK_TABLES_UPDATE_RATE);

    oi = new OI();

    try
    {
        Logger.CustomLogger.setup();
    }
    catch (Throwable e) { Logger.logException(e);}
    
    //Logger.consoleLog();

    SmartDashboard.putData("Auto mode", auton_chooser);
    //Logger.consoleLog();
    System.out.println("Robot init");

    leds.socketSetup(RobotConstants.LED_PORT);    
    System.out.println("LED Setup Run");

    // autonomousCommand = oi.getSelectedAutonCommand();
    leds.setStillDrive(true);

    elbow.stop();
    wrist.stop();
    extension.stop();

    // operatorControl = new ControlOperatorController(oi.getOperatorController());
    updateArbitraryFeedForwards = new Notifier(new Runnable() {
      public void run() {
        wrist.updateArbitraryFeedForward();
        elbow.updateArbitraryFeedForward();
        extension.updateArbitraryFeedForward();
        intake.updateArbitraryFeedForward();
      }
    });

    updateArbitraryFeedForwards.startPeriodic(0.01);

    SmartDashboard.putNumberArray("reflect_tape_vision_data", new double[]{RobotConstants.VISION_ERROR_CODE,RobotConstants.VISION_ERROR_CODE,RobotConstants.VISION_ERROR_CODE});
    
    SmartDashboard.putString("vision-camera", "front");
    SmartDashboard.putString("vision-enabled", "disabled");
    SmartDashboard.putString("vision-status", "");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */  
 @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("gyro", (int) sensors.getAngle() % 360);
    // SmartDashboard.putString("current-command", Scheduler.getInstance().getName());
    // SmartDashboard.putString("current-arm-state", Arm.getCurrentState().toString());
    // SmartDashboard.putNumber("intake-angle", intake.getAngleInDegrees());
    // SmartDashboard.putNumber("elbow-angle", elbow.getAngleInDegrees());
    // SmartDashboard.putNumber("wrist-angle", wrist.getAngleInDegrees());
    // SmartDashboard.putBoolean("intake-ir-sensor", sensors.getIntakeIROutput());
    // SmartDashboard.putNumber("extension-actual-length" , extension.getLengthInches());
    // SmartDashboard.putNumber("arm-extension" , extension.getLengthInches() / Extension.EXTENSION_OUT_IN_INCHES);
    // // SmartDashboard.putNumber("Actual Extension" , extension.getLengthInches());
    // SmartDashboard.putBoolean("drive-reversed-status", XboxRocketLeagueDrive.isDriveReversed());
    // if (arm.getHeldItem().equals(HeldItem.HATCH)) {
    //   SmartDashboard.putString("claw-status", "Hatch");
    // } else if (arm.getHeldItem().equals(HeldItem.BALL)) {
    //   SmartDashboard.putString("claw-status", "Ball");
    // } else if (arm.getHeldItem().equals(HeldItem.NONE)) {
    //   SmartDashboard.putString("claw-status", "None");
    // } else if (claw.isOpen()) {
    //   SmartDashboard.putString("claw-status", "open");
    // } else if (!claw.isOpen()) {
    //   SmartDashboard.putString("claw-status", "close");
    // }

    // SmartDashboard.putNumber("Yaw", sensors.getYawDouble());
    // // SmartDashboard.putNumber("Phi", sensors.getAngleToTarget());
    // SmartDashboard.putNumber("Horizontal Angle", coprocessor.getAngleToWallTarget());
    // // SmartDashboard.putNumber("Depth", coprocessor.getDistanceToWallTarget());

    // // SmartDashboard.putNumber("Arbitrary Feedforward Measurement", MeasureArbitraryFeedforward.output);


    // elbow.sendDataToDashboard();
    // // extension.sendDataToDashboard();
    // // wrist.sendDataToDashboard();
    // // intake.sendDataToDashboard();
    // sensors.sendBreamBreakDataToDashboard();
    driveBase.sendEncoderDataToDashboard();

  }
  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    //SmartDashboard.putString("robot-state", "disabledPeriodic()");
    Logger.consoleLog("Robot Disabled");
    // autonomousCommand = oi.getSelectedAutonCommand();
    leds.setStillDrive(true);
    // driveBase.initCoastMode();
    intake.stop();
    elbow.stop();
    extension.stop();
    wrist.stop();
  }

  @Override
  public void disabledPeriodic() {

    sensors.sendUltrasonicDataToDashboard();
    //driveBase.sendEncoderDataToDashboard();

    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    driveBase.resetOdometry();
   // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(RobotConstants.kMaxSpeedMetersPerSecond, RobotConstants.kMaxAccelerationMetersPerSecondSquared)
       // Add kinematics to ensure max speed is actually obeyed
       .setKinematics(RobotConstants.kDriveKinematics).addConstraint(RobotConstants.kAutoPathConstraints);

// An example trajectory to follow.  All units in meters.
Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
   // Start at the origin facing the +X direction
   new Pose2d(0, 0, new Rotation2d(0)),
   // Pass through these two interior waypoints, making an 's' curve path
   List.of(
      new Translation2d(3, 0)
     // new Translation2d(2, 2),
     // new Translation2d(4, -2)
   ),
   // End 3 m straight ahead of where we started, facing forward
   new Pose2d(6, 0, new Rotation2d(0)),
   // Pass config
   config
    );

    System.out.println(exampleTrajectory.toString());
  // DriveBase extends wpilibj.command.Subsystem whereas the v2 Command infrastructure
  // requires a class that implements wpilibj2.command.Subsystem. So for now just passing a null.
  // If we go down this road, we'll have to buy in to v2 Commands or backport this Command
  edu.wpi.first.wpilibj2.command.Subsystem meetingTheRequirement = null;

  RamseteCommand ramseteCommand = new RamseteCommand(
    exampleTrajectory,
    driveBase::getPose,
    new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta),
    RobotConstants.ksVolts,
    RobotConstants.kvVoltSecondsPerMeter,
    RobotConstants.kaVoltSecondsSquaredPerMeter,
    RobotConstants.kDriveKinematics,
    driveBase.getLeftEncoder()::getVelocity,
    driveBase.getRightEncoder()::getVelocity,
    new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel, RobotConstants.kDDriveVel),
    new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel, RobotConstants.kDDriveVel),
    // RamseteCommand passes volts to the callback
    driveBase::tankDriveVoltage,
    meetingTheRequirement);

// Run path following command, then stop at the end.
    autonomousCommand = ramseteCommand.andThen(() -> driveBase.tankDrive(0, 0));
    autonomousCommand.schedule();


    // if (operatorControl != null) {
    //   operatorControl.start();
    // }

    //Scheduler.getInstance().add(new TimeDrive(20, 0.8));
 
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Robot.driveBase.periodic();
    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().run();
   // Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putString("robot-state", "teleopPeriodic()");
    leds.setForwardData(true);
    driveBase.initBrakeMode();

    if(DriverStation.getInstance().getMatchTime() < 130) {
      Scheduler.getInstance().add(new SafelyResetExtension());
    }

    Logger.consoleLog("Teleop Started");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

}