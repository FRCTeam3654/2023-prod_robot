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
public class AutonomousCCommand extends SequentialCommandGroup {
  NewRunMotionProfile mp;
  NewRunMotionProfile mp1;
  /** Creates a new AutonomousRedCCommand. */
  public AutonomousCCommand(RobotOdometry odometry, Drive driveTrain) {
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()), 0,
    List.of(),
    new Pose2d(Units.inchesToMeters(116), Units.inchesToMeters(30), Rotation2d.fromDegrees(0)), 0, false, false);

mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(116), Units.inchesToMeters(30), new Rotation2d()), 0,
    List.of(),
    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, true, false);

addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()))), mp, new WaitCommand(2));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
