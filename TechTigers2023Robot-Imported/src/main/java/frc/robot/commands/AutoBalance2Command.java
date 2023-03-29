// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;


public class AutoBalance2Command extends CommandBase {
  /** Creates a new AutoBalance2Command. */
    private boolean isBackDriveStarted = false;
    private boolean isForwardDriveStarted = false;
    //private double backDriveCount = 0;
    private double backDriveStartTime = 0;
    //private double forwardDriveCount = 0;
    private double forwardDriveStartTime = 0;
    public static double initialPitch = 0; // only set once at the beginning of automonous
    private boolean isAngleReached = false;
    private double reverseTippySpeed;
    private double forwardTippySpeed;
  
    private boolean IsButtonPressed=true;
    private boolean WasButtonNotPressed=false;
  
    double[] yawPitchRollArray = new double[3];

  public AutoBalance2Command() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);

    //initialPitch = yawPitchRollArray[1];
    initialPitch = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);

    double angleDifference = yawPitchRollArray[1] - initialPitch;

    //if ((isAngleReached == false) && (Math.abs(angleDifference) > 8) ){
    //  isAngleReached = true;
    //}

    
    reverseTippySpeed = (RobotMap.balanceAngleM * angleDifference - RobotMap.balanceAngleB);
    forwardTippySpeed = (RobotMap.balanceAngleM * angleDifference + RobotMap.balanceAngleB);
    //System.out.println(reverseTippySpeed + " , " + forwardTippySpeed +", angle = "+angleDifference);
    if(reverseTippySpeed < -0.25){
      reverseTippySpeed = -0.25;  // tune down a little from 0.2
    }
    if(forwardTippySpeed > 0.25){
      forwardTippySpeed = 0.25;
    }

 
    RobotContainer.drive.setPercentOutput(0.3);
  
    //if(angleDifference < 0 && Math.abs(angleDifference) > RobotMap.balanceAngleTolerance && isAngleReached == true){   
    if(angleDifference < 0 && Math.abs(angleDifference) > RobotMap.balanceAngleTolerance ){   
        RobotContainer.drive.setPercentOutput(reverseTippySpeed);    
    }
    //else if(Math.abs(angleDifference) <= RobotMap.balanceAngleTolerance && isAngleReached == true){
    else if(Math.abs(angleDifference) <= RobotMap.balanceAngleTolerance ){
      RobotContainer.drive.setPercentOutput(0);
    }
    //else if (((angleDifference > RobotMap.balanceAngleTolerance && isAngleReached == true))) {
    else if (((angleDifference > RobotMap.balanceAngleTolerance ))) {
      RobotContainer.drive.setPercentOutput(forwardTippySpeed);
      //System.out.println("am i reversing");
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
