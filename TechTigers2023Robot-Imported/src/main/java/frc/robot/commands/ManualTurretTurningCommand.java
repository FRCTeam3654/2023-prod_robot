// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;


public class ManualTurretTurningCommand extends CommandBase {
  /** Creates a new TurretTurningCommand. */


  public ManualTurretTurningCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSpark);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.turretSpark.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //RobotContainer.oi.turretButton.
    if(RobotContainer.oi.turretRightPOV.getAsBoolean() == true){
      RobotContainer.turretSpark.manualTurretControl(0.2 * RobotContainer.turretSpark.maxRPM);
    }

    else if(RobotContainer.oi.turretLeftPOV.getAsBoolean() == true){
      RobotContainer.turretSpark.manualTurretControl(-0.2 * RobotContainer.turretSpark.maxRPM);
    }
    
    
/* 
     else if(RobotContainer.oi.limelightButton.getAsBoolean() )  {
        
        // drive towards the april tag 
        if ( hasTargets == true) {
          joystickRightX = (-1) * yawFromAprilTag * RobotMap.driveToAprilTagProportion;
          driveStraightFlag = false;
          System.out.println("AprilTag Button is clicked ... joystickX  ="+joystickRightX +", yawFromAprilTag = "+yawFromAprilTag);
        }
        else {
          if (hasTargetEver == true) {
            double vinniesError = driveStraightAngleByAprilTag - yawPitchRollArray[0];
            joystickRightX = vinniesError * RobotMap.driveStraightProportion;
            driveStraightFlag = true;
          }
          else {
            driveStraightFlag = false;
          }
        }
      }
      else if(!RobotContainer.oi.limelightButton.getAsBoolean()){
        driveStraightFlag = false;
      }
    */
    else {
      RobotContainer.turretSpark.manualTurretControl(0);
    }

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turretSpark.manualTurretControl(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (RobotContainer.turret.turretTickCount() >= RobotMap.turretTickAmount){
      //RobotContainer.turret.zeroSensor();
      //return true;
    //}
    return false;
  }
}
