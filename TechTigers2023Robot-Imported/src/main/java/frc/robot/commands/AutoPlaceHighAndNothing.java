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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceHighAndNothing extends SequentialCommandGroup {

  NewRunMotionProfile mp;
  NewRunMotionProfile mp1;

  /** Creates a new AutoPlaceHighCubeAndMove. */

  // the only difference from place mid is that place high need roller (instead of Pneumatics) to shoot the cube to the platform

  public AutoPlaceHighAndNothing(RobotOdometry odometry, Drive driveTrain) {

    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()), 0,
      List.of(),
      new Pose2d(Units.inchesToMeters(-160), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, true, false);

    mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(-150), Units.inchesToMeters(0), new Rotation2d()), 0,
      List.of(),
      new Pose2d(Units.inchesToMeters(-65), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, false, false);

      addCommands(
          new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(0),  Units.inchesToMeters(0), new Rotation2d()))) ,
          /*new ParallelCommandGroup(
              new ArmSetPositionsCommand(3, 2200, 1.0)
              new  SequentialCommandGroup (
                  new WaitCommand(0.3),
                  new AutoArmJoustCommand(1)  // NEW: 2 seconds for telescoping arm to extend
              )
              
          ),
          */
          new ParallelCommandGroup(
            new ArmSetPositionsCommand(1, 2), // raise arm to full distance, 2 seconds
            new  SequentialCommandGroup (
                  new WaitCommand(0.5), // vs 0.8
                  new  AutoWrist(1, 1.1 * RobotMap.wristFullUpDistance)// lowers wrist , 1.5 seconds
            )
          ),
          //new AutoPneumatics(1, 0.5),
          // --- shoot the cube via roller here ---
          new AutoIntakeWheelsCommand(3),
          new ParallelCommandGroup(
            new AutoWrist(2), // raise wrist, 1.5 seconds, don't wait for full 2 seoonds to do next command
            //new AutoArmJoustCommand(2),    // NEW: 2 seconds for telescoping arm to retract
            //new AutoPneumatics(2),  // 1 second
            new AutoIntakeWheelsCommand(0),  
            new  SequentialCommandGroup (
              new WaitCommand(0.5),
              new ArmSetPositionsCommand(2, 2.0) // lower arm to near bottom, 2 seconds 
            ) //,
            /*new  SequentialCommandGroup(
                mp,                           // estimate about 4 seconds: 1.3 meter/second x 4 = 5.2 meter (~157 inches), after ~ 4 seconds in autonomous
                new WaitCommand(0.8),   // wait for 0.8 second for the balance swing back to nornal
                mp1,                          // drive towards the platform via mp instead of percent output in autobalance
                new AutoBalance2Command()       // about 5 to 6 seconds left to auto balance
            )
            */
          )


      );

    
  }
}
