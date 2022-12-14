// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.PneumaticsTesting;
import frc.robot.commands.ManualDriveLCommand;
import frc.robot.subsystems.RobotOdometry;
//import frc.robot.subsystems.*;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static Drive drive;
  public static PneumaticsTesting pneumaticsTesting;
  public static OI oi;

  private RobotOdometry odometry;

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private final SendableChooser<Command> autoChooser1 = new SendableChooser<Command>();

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  PS4Controller p_driverController = new PS4Controller(OIConstants.kDriverControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // VERY IMPORTANT:   drive need be created before oi since oi creates Turn90DegreesCommand object in which need drive object
    drive = new Drive();
    //pneumaticsTesting = new PneumaticsTesting();
    oi = new OI();  // need be after drive object
    //always keep OI last

    // Configure the button bindings
    configureButtonBindings();

    drive.resetEncoders();
    drive.resetHeading();
    odometry = new RobotOdometry(drive, drive.getPigeonIMU());
    odometry.resetOdometry();
    drive.setDefaultCommand(new ManualDriveRCommand());

    CameraServer.startAutomaticCapture(0);

    //autoChooser.addOption("Auto (Shot and Move)", new NewCommandGroup(odometry, drive));
        
    //autoChooser.addOption("Galactic Search (A)", new RunGalacticSearchA(odometry, drive));
    //autoChooser.addOption("Galactic Search (B)", new RunGalacticSearchB(odometry, drive));
    

    autoChooser.addOption("1 High Goal", new AutonomousACommand(odometry, drive));
    //autoChooser.addOption("2 Ball C", new AutonomousCCommand(odometry, drive));
    //autoChooser.addOption("2 Ball B", new AutonomousBCommand(odometry, drive));
    //autoChooser.setDefaultOption("MoveAndShootLow", new AutonomousDCommand(odometry, drive));
    //autoChooser.addOption("MoveAndShootLow", new AutonomousDCommand(odometry, drive));
    //autoChooser.setDefaultOption("Complicated Auto Route", new HapMapAutoRoute(odometry, drive));

    autoChooser1.setDefaultOption("Right Joysitck Only", new ManualDriveRCommand());
    autoChooser1.addOption("Right Y Left X Joystick", new ManualDriveLRCommand());
    autoChooser1.addOption("Left Joystick Only", new ManualDriveLCommand());
    
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("Drive Control", autoChooser1);

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
    return autoChooser.getSelected();
  }

}
