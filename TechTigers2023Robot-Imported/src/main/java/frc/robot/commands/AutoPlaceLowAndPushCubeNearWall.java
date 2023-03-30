// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;

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
import frc.robot.commands.NewRunMotionProfile.CirclePath;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.DriverStation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceLowAndPushCubeNearWall extends SequentialCommandGroup {

  DriverStation.Alliance color = DriverStation.getAlliance();
  
  NewRunMotionProfile mp;
  NewRunMotionProfile mp1;

  /** Creates a new AutoPlaceLowAndPushCubeLeft. */
  // facing drive station, the left side of charge station
  public AutoPlaceLowAndPushCubeNearWall(RobotOdometry odometry, Drive driveTrain) {
    int multiplier = 1;
    if(color == DriverStation.Alliance.Blue){
        multiplier = 1;
    }
    else {
        multiplier = -1; // red alliance's path is the mirror image of blue, need flip the sign
    }

    // NEED experiment:  when is positive angle ? and postive Y ? when moving backward
    /*
   *   Test finding: 
   *   1. positive or negative Y during driving backward is the same as driving forward:  left side is positive
   * 
   *   2. positive or negative Angle during driving backward is the same as driving forward:  counter clockwise is postive relative to the positive X axis
   * 
   */


    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
      List.of(new Pose2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), new Rotation2d()),
          new Pose2d(Units.inchesToMeters(-80.0), Units.inchesToMeters(multiplier * 20.0), new Rotation2d()), 
          new Pose2d(Units.inchesToMeters(-160.0), Units.inchesToMeters(0.0), new Rotation2d()), 
          new CirclePath(new Translation2d(Units.inchesToMeters(-209), Units.inchesToMeters(0)), Units.inchesToMeters(24), new Rotation2d( multiplier * (-45)), Rotation2d.fromDegrees( multiplier * (-90)), true),
          new CirclePath(new Translation2d(Units.inchesToMeters(-209), Units.inchesToMeters(multiplier * (-48))), Units.inchesToMeters(24), new Rotation2d(multiplier * (90)), Rotation2d.fromDegrees(multiplier * 145), false),
          new Pose2d(Units.inchesToMeters(-216.0), Units.inchesToMeters(multiplier *(-31.0)), new Rotation2d(multiplier * 50)) 
          ),
      0.0, true, false);

 

    mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(-216), Units.inchesToMeters(multiplier *(-31)), new Rotation2d(multiplier * 50)), 0,
        List.of(new Translation2d(Units.inchesToMeters(-209), Units.inchesToMeters(0)),
                new Translation2d(Units.inchesToMeters(-177), Units.inchesToMeters(multiplier * 16)),
                new Translation2d(Units.inchesToMeters(-150), Units.inchesToMeters(multiplier * 16))
        
        ),
        new Pose2d(Units.inchesToMeters(-65), Units.inchesToMeters(multiplier * 16), Rotation2d.fromDegrees(0)), 1.3, false, false);



        addCommands(
          new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(0),  Units.inchesToMeters(0), new Rotation2d()))),
          new ParallelCommandGroup(
            new ArmSetPositionsCommand(2000), // raise arm a little bit, 1.5 seconds
            new  AutoWrist(1), // lowers wrist , 1.5 seconds
            new  SequentialCommandGroup (
              new WaitCommand(0.4),
              new AutoPneumatics(1)
              //new IntakeWheelsCommand(2)
            )
          ),
          new ParallelCommandGroup(
            new AutoWrist(2), // raise wrist, 1.5 seconds, don't wait for full 1.5 seoonds to do next command
            new  SequentialCommandGroup (
              new WaitCommand(0.5),   // wait for 0.5 second for wrist to raise above group
              new ParallelCommandGroup(
                new AutoPneumatics(2),  // 1 second
                //new IntakeWheelsCommand(0),
                new  SequentialCommandGroup(
                    mp,                           // estimate about 4 seconds: 1.3 meter/second x 4 = 5.2 meter (~157 inches), after ~ 4 seconds in autonomous
                    new WaitCommand(0.2),   // wait for 0.2 second for robot to stablize
                    mp1,                          // drive towards the cube and push it to the low, about 4 seconds
                    new AutoDriveWithTimeout()       // drive straight at constant percent power for 2 seconds 
                )
              )
            )
          )
     );


  }
}
