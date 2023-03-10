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
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoWrist;
import frc.robot.commands.AutoPneumatics;
import frc.robot.commands.ArmSetPositionsCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceLowAndMove extends SequentialCommandGroup {
  /** Creates a new AutoStayStillCommand. */

  NewRunMotionProfile mp;
  public AutoPlaceLowAndMove(RobotOdometry odometry, Drive driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(-170), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, true, false);

    addCommands(
      new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(0),  Units.inchesToMeters(0), new Rotation2d()))), 
      //new ArmSetPositionsCommand(2000), new AutoWrist(1), new AutoPneumatics(1), new AutoWrist(1), new AutoPneumatics(1), 
      new ArmSetPositionsCommand(2000), new AutoWrist(1), new AutoPneumatics(1), new AutoWrist(2), new AutoPneumatics(2),
      mp
    );

  }
}
