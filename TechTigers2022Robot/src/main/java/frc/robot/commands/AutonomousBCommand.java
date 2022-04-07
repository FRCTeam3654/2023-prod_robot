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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class AutonomousBCommand extends SequentialCommandGroup {
  NewRunMotionProfile mp;
  NewRunMotionProfile mp1;
  /** Creates a new AutonoumousRedBCommand. */
  public AutonomousBCommand(RobotOdometry odometry, Drive driveTrain) {
    /*mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()), 0,
        List.of(),
        //new Pose2d(Units.inchesToMeters(93), Units.inchesToMeters(85), Rotation2d.fromDegrees(0)), 0, false, false);
        new Pose2d(Units.inchesToMeters(93), Units.inchesToMeters(-35), Rotation2d.fromDegrees(-90)), 0, false, false);*/
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(35), Units.inchesToMeters(30), new Rotation2d(0)), 0,
    List.of(),
    new Pose2d(Units.inchesToMeters(85), Units.inchesToMeters(30), Rotation2d.fromDegrees(0)), 0, false, false);


    //mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(93), Units.inchesToMeters(85), new Rotation2d()), 0,
    /*mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(93), Units.inchesToMeters(-35), new Rotation2d(-90)), 0,

        List.of(),
        new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, true, false);
        */

  //SlidingClimbHooksCommand.climbNumber = 1;

    //addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()))), new SlidingClimbHooksCommand(1),new BeltcroShooterCommand(), mp, new WaitCommand(2), mp1, new BeltcroShooterCommand());
    addCommands(
  new ParallelDeadlineGroup(
    new SequentialCommandGroup(
      new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(35), Units.inchesToMeters(30), new Rotation2d())))
      )
    ,
    mp)
);
  }
}
