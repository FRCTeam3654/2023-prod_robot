// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;
import frc.robot.commands.NewRunMotionProfile.CirclePath;
//import frc.robot.Constants;
//import frc.robot.Constants.RobotType;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HapMapAutoRoute extends SequentialCommandGroup {
  NewRunMotionProfile forward194TurnLeft90;
  NewRunMotionProfile forward165;
  NewRunMotionProfile back100TurnRight90;
  NewRunMotionProfile fig8x1;
  NewRunMotionProfile fig8x2;
  NewRunMotionProfile fig8x3;
  NewRunMotionProfile fig8x4;
  NewRunMotionProfile fig8x5;
  NewRunMotionProfile fig8x6;
  NewRunMotionProfile fig8x7;
  NewRunMotionProfile fig8x8;
  NewRunMotionProfile forward100TurnRight90;
  /** Creates a new AutonomousDCommand. */
  public HapMapAutoRoute(RobotOdometry odometry, Drive driveTrain) {
    forward194TurnLeft90 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(194), Units.inchesToMeters(0), Rotation2d.fromDegrees(90)), 0, false, false);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

   forward165 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(194), Units.inchesToMeters(0), new Rotation2d(90)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(194), Units.inchesToMeters(165), Rotation2d.fromDegrees(90)), 0, false, false);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

    back100TurnRight90 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(194), Units.inchesToMeters(165), new Rotation2d(90)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(194), Units.inchesToMeters(65), Rotation2d.fromDegrees(0)), 0, true, false);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

    fig8x1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(194), Units.inchesToMeters(65), new Rotation2d(0)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(269), Units.inchesToMeters(-10), Rotation2d.fromDegrees(0)), 0, false, false);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //fig8x2 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(269), Units.inchesToMeters(-10), new Rotation2d(0)), 0,
        //List.of(),
        //new Pose2d(Units.inchesToMeters(344), Units.inchesToMeters(65), Rotation2d.fromDegrees(90)), 0, false, false);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

    fig8x3 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(269), Units.inchesToMeters(-10), new Rotation2d(0)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(419), Units.inchesToMeters(140), Rotation2d.fromDegrees(0)), 0, false, false);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

    fig8x4 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(419), Units.inchesToMeters(140), new Rotation2d(0)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(494), Units.inchesToMeters(65), Rotation2d.fromDegrees(-90)), 0, false, false); 
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
    fig8x5 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(494), Units.inchesToMeters(65), new Rotation2d(-90)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(419), Units.inchesToMeters(-10), Rotation2d.fromDegrees(-180)), 0, false, false);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //fig8x6 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(419), Units.inchesToMeters(-10), new Rotation2d(-180)), 0,
        //List.of(),
        //new Pose2d(Units.inchesToMeters(344), Units.inchesToMeters(65), Rotation2d.fromDegrees(-270)), 0, false, false);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

   fig8x7 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(419), Units.inchesToMeters(-10), new Rotation2d(-180)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(269), Units.inchesToMeters(140), Rotation2d.fromDegrees(-180)), 0, false, false);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
   fig8x8 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(269), Units.inchesToMeters(140), new Rotation2d(-180)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(194), Units.inchesToMeters(65), Rotation2d.fromDegrees(-180)), 0, false, false);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

    forward100TurnRight90 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(194), Units.inchesToMeters(65), new Rotation2d(-180)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(194), Units.inchesToMeters(165), Rotation2d.fromDegrees(90)), 0, false, false);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(0),  Units.inchesToMeters(0), new Rotation2d()))), forward194TurnLeft90, forward165, new WaitCommand(10), back100TurnRight90,
          fig8x1, fig8x3, fig8x4, fig8x5, fig8x7, fig8x8, forward100TurnRight90, new WaitCommand(2), new BallShooterCommand(1,1))),
      
        new IntakeStopCommand()
    );
}
}

//75x75
