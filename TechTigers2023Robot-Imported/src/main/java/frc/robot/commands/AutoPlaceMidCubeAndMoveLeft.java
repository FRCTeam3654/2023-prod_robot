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
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.WristMotionMagic;
import frc.robot.commands.PneumaticsGrabbingCommand;
import frc.robot.commands.ArmSetPositionsCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ArmJoustCommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class AutoPlaceMidCubeAndMoveLeft  extends SequentialCommandGroup {

  NewRunMotionProfile mp;

  public AutoPlaceMidCubeAndMoveLeft (RobotOdometry odometry, Drive driveTrain) {

    // assume the left is the positive Y (robot front facing driver station)
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()), 0,
        List.of(new Translation2d(Units.inchesToMeters(-20), Units.inchesToMeters(9)), new Translation2d(Units.inchesToMeters(-40), Units.inchesToMeters(18))),
        new Pose2d(Units.inchesToMeters(-160), Units.inchesToMeters(18), Rotation2d.fromDegrees(0)), 0, true, false);
   

   
        addCommands(
            new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(0),  Units.inchesToMeters(0), new Rotation2d()))) ,
        
            new ParallelCommandGroup(
              new ArmSetPositionsCommand(1, 2.5), // raise arm to full distance, 2 seconds
              new  SequentialCommandGroup (
                new WaitCommand(0.3),   
                new ParallelCommandGroup(
                  new ArmJoustCommand(1),  // NEW: 2 seconds for telescoping arm to extend
                  new  SequentialCommandGroup (
                    new WaitCommand(0.8), // adjust this value: the exact time the arm above the cube platform
                    new  AutoWrist(1)// lowers wrist , 1.5 seconds
                  )
                )
              )
            ),
            new AutoPneumatics(1),
            new ParallelCommandGroup(
              new AutoWrist(2), // raise wrist, 1.5 seconds, don't wait for full 2 seoonds to do next command
              new ArmJoustCommand(2),  // NEW: 2 seconds for telescoping arm to retract
              new  SequentialCommandGroup (
                new WaitCommand(1),   // wait for 1 second for wrist to raise above group
                new ArmSetPositionsCommand(2, 2.0), // lower arm to near bottom, 2 seconds
                new ParallelCommandGroup(
                  new AutoPneumatics(2),  // 1 second   
                  mp                           // estimate about 4 seconds: 1.3 meter/second x 4 = 5.2 meter (~157 inches), after ~ 4 seconds in autonomous
                )
              )
            )

        );


  }

}
