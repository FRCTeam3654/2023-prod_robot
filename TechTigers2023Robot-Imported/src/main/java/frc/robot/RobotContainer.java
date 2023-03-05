// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static Drive drive;
  public static Turret turret;
  public static TurretSpark turretSpark;
  public static PneumaticGrab pneumaticGrab;
  public static TelescopingArm telescopingArm;
  public static VerticalMotionArm verticalMotionArm;
  public static Wrist wrist;
  public static OI oi;
  public static double initialPitch;
  double[] yawPitchRollArray;

  boolean allianceColor;


  private RobotOdometry odometry;

  //private final SendableChooser<Command> autoRedChooser = new SendableChooser<Command>();
  //private final SendableChooser<Command> autoBlueChooser = new SendableChooser<Command>();
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();


  private final SendableChooser<Command> driveChooser = new SendableChooser<Command>();

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  //XboxController m_operatorController = new XboxController(OIConstants.kDriverControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // VERY IMPORTANT:   drive need be created before oi since oi creates Turn90DegreesCommand object in which need drive object
    drive = new Drive();
    turret = new Turret();
    turretSpark = new TurretSpark();
    telescopingArm = new TelescopingArm();
    verticalMotionArm = new VerticalMotionArm();
    wrist = new Wrist();
    oi = new OI();
    pneumaticGrab = new PneumaticGrab();
      // need be after drive object
    //always keep OI last

    // Configure the button bindings
    configureButtonBindings();

    drive.resetEncoders();
    drive.resetHeading();
    odometry = new RobotOdometry(drive, drive.getPigeonIMU());
    odometry.resetOdometry();
    drive.setDefaultCommand(new BothJoystickDriveCommand());
    pneumaticGrab.setDefaultCommand(new PneumaticsGrabbingCommand());
    //wrist.setDefaultCommand(new ManualWristCommand());
    //verticalMotionArm.setDefaultCommand(new ManualVerticalArmCommand());

    CameraServer.startAutomaticCapture(0);


    autoChooser.setDefaultOption("RED move forward and balance", new AutoBalanceRouteCommand(odometry, drive));
    autoChooser.setDefaultOption("BLUE move forward and balance", new AutoBalanceRouteCommand(odometry, drive));
    
    driveChooser.addOption("Left Joystick Drive", new ManualDriveCommand());
    driveChooser.setDefaultOption("Both Joystick Drive", new BothJoystickDriveCommand());
    driveChooser.addOption("Right Joystick Drive", new ManualDriveRightCommand());
    
    SmartDashboard.putData("Drive Mode", driveChooser);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    //get a reference to the subtable called "datatable"
    NetworkTable fmsInfo = inst.getTable("FMSInfo");

    



    yawPitchRollArray = new double[3];
    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);
    initialPitch = yawPitchRollArray[1];


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
   
  }

  public Command getAutonomousCommand() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);
    
    return autoChooser.getSelected();
  }


  //public double getLeftTriggerAxis() {
   // return m_hid.getLeftTriggerAxis();
 // }

  public Command getDriveModeCommand(){
    return driveChooser.getSelected();
  }

}
