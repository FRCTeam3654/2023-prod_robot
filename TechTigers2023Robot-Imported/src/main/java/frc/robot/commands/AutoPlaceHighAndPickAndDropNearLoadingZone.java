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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceHighAndPickAndDropNearLoadingZone extends SequentialCommandGroup {

  DriverStation.Alliance color = DriverStation.getAlliance();

  NewRunMotionProfile mp;
  NewRunMotionProfile mp1;

  /** Creates a new AutoPlaceLowAndPushCubeRight. */
   // facing drive station, the right side of charge station
  public AutoPlaceHighAndPickAndDropNearLoadingZone(RobotOdometry odometry, Drive driveTrain) {

    int multiplier = 1;
    if(color == DriverStation.Alliance.Blue){
      multiplier = 1;
    }
    else {
      multiplier = -1; // red alliance's path is the mirror image of blue, need flip the sign
    }

	
/****   test code to be removed in real field ***/
multiplier = 1;

	
	
    // NEED experiment:  when is positive angle ? and postive Y ? when moving backward
    
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)), 0,
    List.of()
    ,
    new Pose2d(Units.inchesToMeters(-155), Units.inchesToMeters(multiplier * (-4.8)), Rotation2d.fromDegrees(0)), 0, true, false);


    mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(-155), Units.inchesToMeters(multiplier * (-4.8)), new Rotation2d(0)), 0,
    List.of()
    ,
    new Pose2d(Units.inchesToMeters(-8), Units.inchesToMeters(multiplier * (-20)), Rotation2d.fromDegrees(0)), 0, false, false);

    
    
    
    

        addCommands(

            new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(0),  Units.inchesToMeters(0), new Rotation2d()))) ,
            
            new ParallelCommandGroup(
            new AutoArmSetPositionsCommand(1, 1.2 * RobotMap.armFullUpDistance, 2), // raise arm to full distance, 2 seconds
                new  SequentialCommandGroup (
                  new WaitCommand(0.1), // vs 0.5
                  new  AutoWrist(1, 1.5 * RobotMap.wristFullUpDistance)// lowers wrist , 1.5 seconds
                )
            ),
    
            new WaitCommand(1) ,  // give the arm a little time to stablize
            
            
            new ParallelCommandGroup(
                new AutoIntakeWheelsCommand(3),
                new  SequentialCommandGroup(      
                  new WaitCommand(0.5), 
                  new ParallelCommandGroup(
                        new AutoWrist(2), // raise wrist, 1.5 seconds, don't wait for full 2 seoonds to do next command
                        new AutoIntakeWheelsCommand(0),  
                        new  SequentialCommandGroup (
                          new WaitCommand(0.5),
                          new AutoArmSetPositionsCommand(2, 2200, 2.0) // lower arm to near bottom, 2 seconds 
                        ),
                        new  SequentialCommandGroup(   

                              new ParallelCommandGroup(
                                mp,                   // estimate about 4 seconds: 1.3 meter/second x 4 = 5.2 meter (~157 inches), after ~ 4 seconds in autonomous    
                                new  SequentialCommandGroup (
                                  new WaitCommand(0.5),
                                  new AutoTurrentTurningCommand(4,2.0, -5.6) ,      // turn turret 180 toward right (-5.6), ~ 2 seconds
                                  new ParallelCommandGroup(			
                                      new IntakeWheelsCommand(1)  ,  // intake
                                      new  SequentialCommandGroup (
                                        new WaitCommand(1), // experiment wait time to make sure to lower the wrist at the end 
                                        new AutoWrist(4)                        // low wrist , 1.5 second
                                      )
                                  )
                                )
                              ),

                              new ParallelCommandGroup(	
                                mp1,                // drive towards the goal to drop the game piece
                                new IntakeWheelsCommand(0)  ,  
                                new AutoWrist(2)    ,           // raise wrist , 1.5 second
                                new AutoTurrentTurningCommand(4,2.0, 0) // turret go home    
                              ),
                              
                              new ParallelCommandGroup(
                                new AutoWrist(3),
                                new  SequentialCommandGroup (
                                  new WaitCommand(0.5),
                                  new AutoIntakeWheelsCommand(2)
                                )
                              ),

                              new IntakeWheelsCommand(0)  

                        )
                        
                  )
             
                )
            )
            

            
     );


  }
}
