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
public class PlaceMidCommand extends SequentialCommandGroup {
  /** Creates a new PlaceMidCommand. */
  public PlaceMidCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
          new ParallelCommandGroup(
              new ArmSetPositionsCommand(3, 2200, 1.0),
              new  SequentialCommandGroup (
                  new WaitCommand(0.3),
                  new AutoArmJoustCommand(1)  // NEW: 2 seconds for telescoping arm to extend
              )
          ),
          new ParallelCommandGroup(
            new ArmSetPositionsCommand(1, 2), // raise arm to full distance, 2 seconds
            //new  SequentialCommandGroup (
              //new ParallelCommandGroup(
                new  SequentialCommandGroup (
                  new WaitCommand(0.5), // vs 0.8
                  new  AutoWrist(1, 1.1 * RobotMap.wristFullUpDistance)// lowers wrist , 1.5 seconds
                )
              //)
            //)
          ),
          new AutoPneumatics(1, 0.5),
          //new IntakeWheelsCommand(2),
          new ParallelCommandGroup(
            new AutoWrist(2), // raise wrist, 1.5 seconds, don't wait for full 2 seoonds to do next command
            new AutoArmJoustCommand(2),    // NEW: 2 seconds for telescoping arm to retract
            new AutoPneumatics(2),  // 1 second
            //new IntakeWheelsCommand(0),
            new  SequentialCommandGroup (
              new WaitCommand(0.5),
              new ArmSetPositionsCommand(2, 2.0) // lower arm to near bottom, 2 seconds 
            )
          )

      );
  }
}
