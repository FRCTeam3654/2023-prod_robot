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
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.WristMotionMagic;
import frc.robot.commands.PneumaticsGrabbingCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmSetPositionsCommand;
import frc.robot.commands.AutoPneumatics;
import frc.robot.commands.AutoWrist;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class GetFromHumanPlayer extends SequentialCommandGroup {

 // NewRunMotionProfile mp;
  //NewRunMotionProfile mp1;

  /** Creates a new AutonBalanceCommand. */
  public GetFromHumanPlayer() {
   // addRequirements(RobotContainer.wrist);
  //  addRequirements(RobotContainer.pneumaticGrab);
    //mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()), 0,
        //List.of(),
        //new Pose2d(Units.inchesToMeters(-160), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, true, false);

    //mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(-150), Units.inchesToMeters(0), new Rotation2d()), 0,
       // List.of(),
       // new Pose2d(Units.inchesToMeters(-100), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, false, false);
        //NEED TO CHANGE THE VALUE IT SHOULD NOT GO ALL THE WAY BACK TO 0,0
        
       // mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)), 0,
        //List.of(),
        //new Pose2d(Units.inchesToMeters(-62.5), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, true, false);

        //TEST VALUE BECAUSE WE HAVE SMALL AREA TO TEST RN:
        //new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, true, false);

    //addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()))), mp, new WaitCommand(2), mp1);

   //SlidingClimbHooksCommand.climbNumber = 1;
       //addCommands(
             //new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(0),  Units.inchesToMeters(0), new Rotation2d()))),
              //new ArmSetPositionsCommand(2000), new AutoWrist(1), new AutoPneumatics(1), new AutoWrist(2), new AutoPneumatics(2),
              //mp, new WaitCommand(0.5), new AutoBalanceCommand());

          //addCommands(new WaitCommand(1));

          
 //if(RobotContainer.oi.wristGrabDownButton.getAsBoolean()){
          addCommands(   
            new ParallelCommandGroup(
              new AutoPneumatics(1),
              //new IntakeWheelsCommand(1),
              //new  SequentialCommandGroup (
                //new WaitCommand(0.5),
                new AutoWrist(5)
              //)
            )
            );
            //}

            /* 
else if(RobotContainer.oi.wristGrabUpButton.getAsBoolean()){
            addCommands(
              new ParallelCommandGroup(
                  new AutoWrist(2), // raise wrist, 2 seconds, don't wait for full 2 seoonds to do next command
                  new  SequentialCommandGroup (
                    new WaitCommand(1),   // wait for 1 second for wrist to raise above group
                    //new ParallelCommandGroup(
                      new AutoPneumatics(2)  // 1 second
                    //)
                  )
                )
              );
      }
      */
          //addCommands(new AutoPneumatics(1 )); // opens pnematic to drop, 1 second
      
           /*addCommands(
                new ParallelCommandGroup(
                  new AutoWrist(2), // raise wrist, 2 seconds, don't wait for full 2 seoonds to do next command
                  new  SequentialCommandGroup (
                    new WaitCommand(1),   // wait for 1 second for wrist to raise above group
                    //new ParallelCommandGroup(
                      new AutoPneumatics(2)  // 1 second
                    //)
                  )
                )
           );
           */
           
           
  }

  
}
