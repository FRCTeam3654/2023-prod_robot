// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;


public class BalanceCommand extends CommandBase {
  /** Creates a new BalanceCommand. */
  private boolean isBackDriveStarted = false;
  private boolean isForwardDriveStarted = false;
  //private double backDriveCount = 0;
  private double backDriveStartTime = 0;
  //private double forwardDriveCount = 0;
  private double forwardDriveStartTime = 0;
  public static double initialPitch = 0; //set once at beginning of auto
  private boolean isAngleReached = false;
  private double reverseTippySpeed;
  private double forwardTippySpeed;

  private boolean IsButtonPressed=true;
  private boolean WasButtonNotPressed=false;

  double[] yawPitchRollArray = new double[3];

  public BalanceCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    //initialPitch = 0;
    initialPitch = AutoBalanceCommand.initialPitch;
    //reverseTippySpeed = RobotMap.reverseTippySpeed;
    //forwardTippySpeed = RobotMap.forwardTippySpeed;

    IsButtonPressed=true;
    WasButtonNotPressed=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double[] yawPitchRollArray;
    //yawPitchRollArray = new double[3];
    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);

    double angleDifference = yawPitchRollArray[1] - initialPitch;

    if ((isAngleReached == false) && (Math.abs(angleDifference) > 4) ){
      isAngleReached = true;
    }

    IsButtonPressed=RobotContainer.oi.balanceButton.getAsBoolean();
      if (!IsButtonPressed) {
        WasButtonNotPressed=true;
      }

    //reverseTippySpeed = -1 * (RobotMap.balanceAngleM * angleDifference - RobotMap.balanceAngleB);
    //forwardTippySpeed = -1 * (RobotMap.balanceAngleM * angleDifference + RobotMap.balanceAngleB);
    
    reverseTippySpeed = -1 * (RobotMap.balanceAngleM * angleDifference - RobotMap.balanceAngleB);
    forwardTippySpeed = -1 * (RobotMap.balanceAngleM * angleDifference + RobotMap.balanceAngleB);
    System.out.println(reverseTippySpeed + " , " + forwardTippySpeed +", angle = "+angleDifference);
    if(reverseTippySpeed < -0.22){
      reverseTippySpeed = -0.22;
    }
    if(forwardTippySpeed > 0.22){
      forwardTippySpeed = 0.22;
    }
    



    //if tipped back drive forward
    //assume that it is a positive angle --> actually sis bot is negative angle
    //if (((angleDifference > RobotMap.balanceAngleTolerance))) {
    if ( angleDifference < 0 && Math.abs(angleDifference) > RobotMap.balanceAngleTolerance ) {
      //if ((((Timer.getFPGATimestamp() - forwardDriveStartTime) > 2) || isForwardDriveStarted == true) ) {
        //if (isForwardDriveStarted == false) {
          //isForwardDriveStarted = true;
          //forwardDriveCount = backDriveCount + 1;
         // forwardDriveStartTime = Timer.getFPGATimestamp();
      // }
        
        RobotContainer.drive.setPercentOutput(reverseTippySpeed);
      //}
    }
    else if(Math.abs(angleDifference) <= RobotMap.balanceAngleTolerance && isAngleReached == true){
      RobotContainer.drive.setPercentOutput(0);
    }
    else if (((angleDifference > RobotMap.balanceAngleTolerance))) {
    //else if(angleDifference < 0 && Math.abs(angleDifference) > RobotMap.balanceAngleTolerance && isAngleReached == true){
      RobotContainer.drive.setPercentOutput(forwardTippySpeed);
    }

      //if( (yawPitchRollArray[1] - initialPitch) < RobotMap.pitchReverseDegree ) {
         //backDriveCount = 0;
      //}

//if tipped forward drive backward
   /*  else {
      if (((angleDifference < RobotMap.pitchReverseDegree) || isBackDriveStarted == true)) {
        reverseTippySpeed = RobotMap.reverseTippySpeed; //need to change to a PID
        if ((((Timer.getFPGATimestamp() - backDriveStartTime) > 2) || isBackDriveStarted == true) ) {
          if (isBackDriveStarted == false) {
            isBackDriveStarted = true;
            //backDriveCount = backDriveCount + 1;
            backDriveStartTime = Timer.getFPGATimestamp();
          }
          RobotContainer.drive.setPercentOutput(reverseTippySpeed);
        }
      }
      else {
        RobotContainer.drive.setPercentOutput(0);// after back for 0.7 s , stop running up to 2 seconds
      }
   }*/


}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!IsButtonPressed && WasButtonNotPressed){
      return true;
    }
    return false;
  }
}
