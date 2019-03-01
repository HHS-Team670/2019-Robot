/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.ControlOperatorController;
import frc.team670.robot.commands.drive.teleop.XboxRocketLeagueDrive;
import frc.team670.robot.commands.drive.vision.VisionPurePursuitWithPivot;
import frc.team670.robot.commands.tuning.ResetPulseWidthEncoder;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangLEDs_2019;
import frc.team670.robot.subsystems.Arm.HeldItem;
import frc.team670.robot.subsystems.elbow.Elbow;
import frc.team670.robot.subsystems.extension.Extension;
import frc.team670.robot.subsystems.wrist.Wrist;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

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
  public static Claw claw = new Claw();
  public static Arm arm = new Arm(elbow, wrist, extension, intake, claw);

  private Notifier updateArbitraryFeedForwards;

  private Command autonomousCommand, operatorControl;
  private SendableChooser<Command> auton_chooser = new SendableChooser<>();
  public static SendableChooser<Boolean> pid_chooser = new SendableChooser<>();
  
  private Timer timer;

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


    oi = new OI();

    try
    {
        Logger.CustomLogger.setup();
    }
    catch (Throwable e) { Logger.logException(e);}
    
    Logger.consoleLog();

    SmartDashboard.putData("Auto mode", auton_chooser);
    Logger.consoleLog();
    System.out.println("Robot init");

    leds.socketSetup(5801);
    System.out.println("LED Setup Run");
    //leds.socketSetup(RobotConstants.LED_PORT);    

    // Setup to receive PID values from smart dashboard
    pid_chooser.setDefaultOption("false", false);
    pid_chooser.addOption("true", true);
    // SmartDashboard.putData("PID Inputs from Dashboard?", pid_chooser);
    // SmartDashboard.putNumber("P", 0);
    // SmartDashboard.putNumber("I", 0);
    // SmartDashboard.putNumber("D", 0);
    // SmartDashboard.putNumber("KA", 0);

    // autonomousCommand = oi.getSelectedAutonCommand();
    timer = new Timer();

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

    // autonomousCommand = new MeasureTrackwidth();
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
    SmartDashboard.putNumber("gyro", (int) sensors.getAngle() % 360);
    // SmartDashboard.putString("current-command", Scheduler.getInstance().getName());
    SmartDashboard.putString("current-arm-state", Arm.getCurrentState().toString());
    SmartDashboard.putNumber("intake-angle", intake.getAngleInDegrees());
    SmartDashboard.putNumber("elbow-angle", elbow.getAngleInDegrees());
    SmartDashboard.putNumber("wrist-angle", wrist.getAngleInDegrees());
    SmartDashboard.putBoolean("intake-ir-sensor", sensors.getIntakeIROutput());
    SmartDashboard.putNumber("extension-actual-length" , extension.getLengthInches());
    SmartDashboard.putNumber("arm-extension" , extension.getLengthInches() / Extension.EXTENSION_OUT_IN_INCHES);
    SmartDashboard.putNumber("Actual Extension" , extension.getLengthInches());
    SmartDashboard.putBoolean("drive-reversed-status", XboxRocketLeagueDrive.isDriveReversed());
    if (arm.getHeldItem().equals(HeldItem.HATCH)) {
      SmartDashboard.putString("claw-status", "Hatch");
    } else if (arm.getHeldItem().equals(HeldItem.BALL)) {
      SmartDashboard.putString("claw-status", "Ball");
    } else if (arm.getHeldItem().equals(HeldItem.NONE)) {
      SmartDashboard.putString("claw-status", "None");
    } else if (claw.isOpen()) {
      SmartDashboard.putString("claw-status", "open");
    } else if (!claw.isOpen()) {
      SmartDashboard.putString("claw-status", "close");
    }

    SmartDashboard.putNumber("Angle", sensors.getAngle());
    SmartDashboard.putNumber("Phi", sensors.getAngleToTarget());
    SmartDashboard.putNumber("Horizontal Angle", coprocessor.getAngleToWallTarget());
    SmartDashboard.putNumber("Depth", coprocessor.getDistanceToWallTarget());

    // SmartDashboard.putNumber("Arbitrary Feedforward Measurement", MeasureArbitraryFeedforward.output);

    // SmartDashboard.putString("Held Item", arm.getHeldItem().toString());

    // elbow.sendDataToDashboard();
    // extension.sendDataToDashboard();
    // wrist.sendDataToDashboard();
    // intake.sendDataToDashboard();
    sensors.sendUltrasonicDataToDashboard();
    driveBase.sendDIOEncoderDataToDashboard();

  }
  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    SmartDashboard.putString("robot-state", "disabledInit()");
    Logger.consoleLog("Robot Disabled");
    // autonomousCommand = oi.getSelectedAutonCommand();
    driveBase.initCoastMode();
    intake.stop();
    timer.stop();
    intake.stop();
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putString("robot-state", "disabledPeriodic()");
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

    leds.setForwardData(true);

    sensors.resetNavX(); // Reset NavX completely, zero the field centric based on how robot faces from start of game.

    // driveBase.initBrakeMode();

    // SmartDashboard.putString("robot-state", "autonomousInit()");

    // if(DriverStation.getInstance().getAlliance().equals(Alliance.Red)) {
    //   leds.changeAlliance(false);
    // } else if (DriverStation.getInstance().getAlliance().equals(Alliance.Blue)) {
    //   leds.changeAlliance(true);
    // } else {
    //   leds.changeAlliance(true);
    // }

    // Logger.consoleLog("Auton Started");
    // timer.start();

    // Scheduler.getInstance().add(new MoveExtensionBackUntilHitsLimitSwitch(extension));
    // arm.setCoastMode();

    // TODO: robot crashing when trying to load path
    // autonomousCommand = oi.getSelectedAutonCommand();
    // autonomousCommand = new RunIntake(intake, sensors, true);
    // schedule the autonomous command (example)

    // autonomousCommand = new VisionPurePursuitWithPivot(driveBase, coprocessor, sensors, 6, true, true);
      // autonomousCommand = new TestVelocityDrive(20, 20);'
    // autonomousCommand = new NavXChangeableAnglePivot(new MutableDouble(15), driveBase, sensors);

    if (autonomousCommand != null) {
      autonomousCommand.start();
    }

    // if (operatorControl != null) {
    //   operatorControl.start();
    // }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // SmartDashboard.putString("robot-state", "autonomousPeriodic()");
    // SmartDashboard.putNumber("game-time", (int) timer.get());
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putString("robot-state", "teleopInit()");
    leds.setForwardData(true);

    driveBase.initBrakeMode();

    leds.setForwardData(true);

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
    SmartDashboard.putString("robot-state", "teleopPeriodic()");
    SmartDashboard.putNumber("game-time", (int) timer.get());
    if (Robot.oi.getDriverController().getYButton()) {
      Scheduler.getInstance().add(new ResetPulseWidthEncoder(wrist));
    }
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

}