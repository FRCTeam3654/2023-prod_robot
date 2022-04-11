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
  NewRunMotionProfile mp2;
  /** Creates a new AutonomousRedCCommand. */
  public AutonomousCCommand(RobotOdometry odometry, Drive driveTrain) {
    /*
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
    */
    /*
  mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(35), Units.inchesToMeters(30), new Rotation2d(0)), 0,
    List.of(),
    new Pose2d(Units.inchesToMeters(85), Units.inchesToMeters(30), Rotation2d.fromDegrees(0)), 0, false, false);
*/
    /*mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(35), Units.inchesToMeters(30), new Rotation2d(0)), 0,
    List.of(),
    new Pose2d(Units.inchesToMeters(116), Units.inchesToMeters(30), Rotation2d.fromDegrees(0)), 0, false, false);
*/
    /*
  mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(35), Units.inchesToMeters(30), new Rotation2d()), 0,
    List.of(),
    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(30), Rotation2d.fromDegrees(0)), 0, true, false);
  
  mp2 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()), 0,
    List.of(),
    new Pose2d(Units.inchesToMeters(-96), Units.inchesToMeters(60), Rotation2d.fromDegrees(0)), 0, true, false);
    */
    
    
     mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(-62.5), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, true, false);
    
    
    mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(-62.5), Units.inchesToMeters(0), new Rotation2d()), 0,
    List.of(new Translation2d(Units.inchesToMeters(-31), Units.inchesToMeters(0)), new Translation2d(Units.inchesToMeters(6), Units.inchesToMeters(-37.5))),
    new Pose2d(Units.inchesToMeters(-31), Units.inchesToMeters(-75), Rotation2d.fromDegrees(135)), 0, false, false);
    
    mp2 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(-31), Units.inchesToMeters(-75), new Rotation2d(135)), 0,
    List.of(new Translation2d(Units.inchesToMeters(6), Units.inchesToMeters(-37.5))),
    new Pose2d(Units.inchesToMeters(-31), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 0, true, false);
    
    
     addCommands(
              new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(0),  Units.inchesToMeters(0), new Rotation2d()))), mp, new BallShooterCommand(1,1), mp1, mp2, new BallShooterCommand(1,1));
 

    
    
//addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(35), Units.inchesToMeters(30), new Rotation2d()))), mp);
//addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(35), Units.inchesToMeters(30), new Rotation2d()))), new SlidingClimbHooksCommand(1), mp, new WaitCommand(2), mp1, new BeltcroShooterCommand());
//addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(35), Units.inchesToMeters(30), new Rotation2d()))), new SlidingClimbHooksCommand(1), mp);

//pulls down orange climb and moves forward synchronously (lol i don't think i spelled that right)
    /*
addCommands(
  new ParallelDeadlineGroup(
   // new SequentialCommandGroup(
      new IntakeCommand(1)
   // )
  , 
  //new SequentialCommandGroup(
   new BeltcroShooterCommand(2)
  //)
  ,
  //  new SequentialCommandGroup(
      //new SlidingClimbHooksCommand(1)
    //  )
   // ,
    new SequentialCommandGroup(
      new WaitCommand(2), new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()))), mp2)
  )
    
);
*/
/*addCommands(
  new ParallelDeadlineGroup(
    new SequentialCommandGroup(
      new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(35), Units.inchesToMeters(30), new Rotation2d()))), new SlidingClimbHooksCommand(1)
      )
    ,
    mp)
);
*/

/*addCommands(
  new ParallelDeadlineGroup(
      new SequentialCommandGroup(
        new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(35),  Units.inchesToMeters(30), new Rotation2d()))), mp, new WaitCommand(2), mp1, new BeltcroShooterCommand()
        )
        ,               
  new IntakeCommand())
);
*/
}
}

  

