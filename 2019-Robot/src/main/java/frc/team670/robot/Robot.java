/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.drive.DriveMotionProfile;
import frc.team670.robot.dataCollection.MustangPi;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.dataCollection.Pose;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Elbow;
import frc.team670.robot.subsystems.Extension;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangLEDs_2019;
import frc.team670.robot.subsystems.Wrist;
import frc.team670.robot.utils.Logger;

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
  public static MustangPi visionPi = new MustangPi();
  public static DriveBase driveBase = new DriveBase();
  private MustangLEDs_2019 leds = new MustangLEDs_2019();
  public static Pose fieldCentricPose = new Pose();

  private long savedTime=0;

  public static Arm arm = new Arm();
  public static Elbow elbow = new Elbow();
  public static Wrist wrist = new Wrist();
  public static Extension extension = new Extension();
  public static Intake intake = new Intake();
  public static Claw claw = new Claw();
  public static Climber climber = new Climber();

  private long periodCount = 0;

  Command autonomousCommand;
  SendableChooser<Command> auton_chooser = new SendableChooser<>();

  public Robot() {

    oi = new OI();

    try
    {
        Logger.CustomLogger.setup();
    }
    catch (Throwable e) { Logger.logException(e);}
    
    Logger.consoleLog();
  }

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // auton_chooser.addDefault("Default Auto", new TimeDrive());
    // chooser.addObject("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", auton_chooser);
    Logger.consoleLog();
    savedTime = System.currentTimeMillis();
    System.out.println("Robot init");

    leds.socketSetup(5801);
    System.out.println("LED Setup Run");
    //leds.socketSetup(RobotConstants.LED_PORT);    
    
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
    // if(System.currentTimeMillis() > savedTime + 1000) {
    //   leds.updateClimbingBoolean(true);
    // }
    // else if(System.currentTimeMillis() > savedTime + 2000) {
    //   leds.updateForwardDrive(true);
    // }
    // else if(System.currentTimeMillis() > savedTime + 3000) {
    //   leds.updateReverseDrive(true);
    // }
    // else if(System.currentTimeMillis() > savedTime + 4000) {
    //   leds.updateVisionData(true);
    //   savedTime = System.currentTimeMillis();
    // }  
      
    // Logger.consoleLog("LeftEncoderPos: %s, RightEncoderPos: %s", driveBase.getLeftDIOEncoderPosition(), driveBase.getRightDIOEncoderPosition());
    // Logger.consoleLog("LeftEncoderVel: %s, RightEncoderVel: %s", driveBase.getLeftDIOEncoderVelocityInches(), driveBase.getRightDIOEncoderVelocityInches());
      
    // if(periodCount % 10 == 0) {
    //   Logger.consoleLog("NavXYawReset: %s, NavXYawFieldCentric: %s", sensors.getYawDouble(), sensors.getFieldCentricYaw());
    // }
      SmartDashboard.putNumber("NavX Yaw", sensors.getYawDouble());

    periodCount ++;
    leds.setClimbingData(true);//we climb

    // System.out.println("Voltage: "+(irSensor.getVoltage()));
    fieldCentricPose.update(); // Update our field centric Pose to the new robot position. Commented out to avoid null-pointers until sensors hooked up.
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    Logger.consoleLog("Robot Disabled");
  }

  @Override
  public void disabledPeriodic() {
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
    sensors.resetNavX(); // Reset NavX completely, zero the field centric based on how robot faces from start of game.
    fieldCentricPose = new Pose();
    Logger.consoleLog("Auton Started");
    autonomousCommand = new DriveMotionProfile("/output/DriveRightCurve.pf1.csv", false);

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {

    Logger.consoleLog("Teleop Started");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // leds.socketSetup(RobotConstants.LED_PORT);
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
