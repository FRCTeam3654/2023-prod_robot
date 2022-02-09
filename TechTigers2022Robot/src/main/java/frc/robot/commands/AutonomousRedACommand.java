// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.commands.NewRunMotionProfile.CirclePath;
//import frc.robot.Constants;
//import frc.robot.Constants.RobotType;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class AutonomousRedACommand extends SequentialCommandGroup {

  NewRunMotionProfile mp;
  NewRunMotionProfile mp1;

  /** Creates a new AutonomousRedACommand. */
  public AutonomousRedACommand(RobotOdometry odometry, Drive driveTrain) {
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(84), Units.inchesToMeters(20), Rotation2d.fromDegrees(0)), 0, false, false);

    mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(84), Units.inchesToMeters(20), new Rotation2d()), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, true, false);

    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()))), mp, new WaitCommand(2),mp1);

   
       /*addCommands(
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
              new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(30),  Units.inchesToMeters(120), new Rotation2d()))), 
                 mp1, mp2, mp3, mp4
                 )
                 ,               
       new BallPickUpCommand(1))
       );*/
  }

  
}
