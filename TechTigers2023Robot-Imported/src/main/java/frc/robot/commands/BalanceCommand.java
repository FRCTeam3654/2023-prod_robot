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
  private double backDriveCount = 0;
  private double backDriveStartTime = 0;
  private double forwardDriveCount = 0;
  private double forwardDriveStartTime = 0;
  private static double initialPitch = 0;
  private double reverseTippySpeed;
  private double forwardTippySpeed;

  private boolean IsButtonPressed=true;
  private boolean WasButtonNotPressed=false;

  public BalanceCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] yawPitchRollArray = new double[3];
    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);
    initialPitch = yawPitchRollArray[1];
    reverseTippySpeed = RobotMap.reverseTippySpeed;
    forwardTippySpeed = RobotMap.forwardTippySpeed;

    IsButtonPressed=true;
    WasButtonNotPressed=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] yawPitchRollArray;
    yawPitchRollArray = new double[3];

IsButtonPressed=RobotContainer.oi.balanceButton.getAsBoolean();
      if (!IsButtonPressed) {
        WasButtonNotPressed=true;
      }

//if tipped backward drive forward
   // if( (yawPitchRollArray[1] - initialPitch) < RobotMap.pitchReverseDegree ) {
     // backDriveCount = 0;
    //}
//if tipped back drive forward
    if ((((yawPitchRollArray[1] - initialPitch) > RobotMap.pitchForwardDegree) || isForwardDriveStarted == true)) {
      forwardTippySpeed = RobotMap.forwardTippySpeed;  //need to change to a PID
      if ((((Timer.getFPGATimestamp() - backDriveStartTime) > 2) || isForwardDriveStarted == true) && (forwardDriveCount < 3) ) {
        if (isForwardDriveStarted == false) {
        isForwardDriveStarted = true;
        forwardDriveCount = backDriveCount + 1;
        forwardDriveStartTime = Timer.getFPGATimestamp();
    }
    RobotContainer.drive.setPercentOutput(forwardTippySpeed);
  }
  if( (yawPitchRollArray[1] - initialPitch) < RobotMap.pitchReverseDegree ) {
    backDriveCount = 0;
  }
else{ //if tipped forward drive backward
  if ((((yawPitchRollArray[1] - initialPitch) < RobotMap.pitchReverseDegree) || isBackDriveStarted == true)) {
    reverseTippySpeed = RobotMap.reverseTippySpeed; //need to change to a PID
    if ((((Timer.getFPGATimestamp() - backDriveStartTime) > 2) || isBackDriveStarted == true) && (backDriveCount < 3) ) {
      if (isBackDriveStarted == false) {
      isBackDriveStarted = true;
      backDriveCount = backDriveCount + 1;
      backDriveStartTime = Timer.getFPGATimestamp();
  }
  RobotContainer.drive.setPercentOutput(reverseTippySpeed);
}
  else {
    RobotContainer.drive.setPercentOutput(0);// after back for 0.7 s , stop running up to 2 seconds
  }
  }
}

}


}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (IsButtonPressed && WasButtonNotPressed){
      return true;
    }
    return false;
  }
}
