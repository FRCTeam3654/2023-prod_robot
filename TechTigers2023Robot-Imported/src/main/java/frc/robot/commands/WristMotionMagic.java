// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;


public class WristMotionMagic extends CommandBase {
  /** Creates a new WristMotionMagic. */
  public static int wristMoveNumber = 0;
  double wristTimer;
  public static boolean isMotionMagicInProgress = false;
  private boolean isDownPOVPressed = false;
  private boolean isUpPOVPressed = false;
  private boolean isMotionMagicButtonPressed = false;
  private double currentPos;
  private int mode = 0;

  public WristMotionMagic() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wrist);
  }

  public WristMotionMagic(int mode){
    addRequirements(RobotContainer.wrist);
    this.mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.wrist.zeroSensor();

    if(RobotContainer.oi.wristDownUpButton.getAsBoolean()  || mode == 1){
      isMotionMagicButtonPressed = true;
    }

    if(isMotionMagicButtonPressed == true){
    wristMoveNumber = wristMoveNumber + 1;

    if(wristMoveNumber %2 == 1 ){ //moves down
      wristTimer = Timer.getFPGATimestamp();
      RobotContainer.wrist.setMotionMagic(-1 * RobotMap.wristFullUpDistance, 4000, 3000);
      //RobotContainer.wrist.setMotionMagic(0, 2000, 2000);
      System.out.println("should i be motion magicking down");
      isMotionMagicInProgress = true;
    }

    else if(wristMoveNumber %2 != 1 ){ //moves up
      wristTimer = Timer.getFPGATimestamp();
      //RobotContainer.wrist.setMotionMagic(RobotMap.wristFullUpDistance, 2000, 2000);
      RobotContainer.wrist.setMotionMagic(0, 8000, 6000);
      System.out.println("should i be motion magicking up");
      isMotionMagicInProgress = true;
    }
    else{
      RobotContainer.wrist.setMotionMagic(0, 0, 0);
      wristTimer = Timer.getFPGATimestamp();
    }
  }

  else if(RobotContainer.oi.wristDownPOV.getAsBoolean()){
    isDownPOVPressed = true;
  }

  if(isDownPOVPressed == true){
    wristTimer = Timer.getFPGATimestamp();
      //RobotContainer.wrist.setMotionMagic(RobotMap.wristFullUpDistance, 2000, 2000);
      currentPos = RobotContainer.wrist.getWristTalonPosition();
      RobotContainer.wrist.setMotionMagic(currentPos + 1000, 2000, 2000);
      System.out.println("should i be motion magicking short down");
      isMotionMagicInProgress = true;
  }


  else if(RobotContainer.oi.wristUpPOV.getAsBoolean()){
    isUpPOVPressed = true;
  }
  if(isUpPOVPressed == true){
    wristTimer = Timer.getFPGATimestamp();
      //RobotContainer.wrist.setMotionMagic(RobotMap.wristFullUpDistance, 2000, 2000);
      currentPos = RobotContainer.wrist.getWristTalonPosition();
      RobotContainer.wrist.setMotionMagic(currentPos - 1000, 2000, 2000);
      System.out.println("should i be motion magicking short up");
      isMotionMagicInProgress = true;
  }
  else{}

//RobotContainer.wrist.setMotionMagic(-13000, 2000, 2000);
//wristTimer = Timer.getFPGATimestamp();
//System.out.println("should i be motion magicking down");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isMotionMagicInProgress = false;
    isMotionMagicButtonPressed = false;
    isDownPOVPressed = false;
    isUpPOVPressed = false;
    mode = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (wristTimer + 2.0) < Timer.getFPGATimestamp()) {
      // after 3 second, stop command
      isMotionMagicInProgress = false;
      return true;
    }
    else {
        double sensorDistance = Math.abs(RobotContainer.wrist.getWristTalonPosition());
        double percentError = 100 * (RobotMap.wristFullUpDistance - sensorDistance)/RobotMap.wristFullUpDistance;

        if (Math.abs(percentError) < 1){
        //if (percentLeftError < 0.9 || percentLeftError < 0 )
        isMotionMagicInProgress = false;
        return true;
        }

  }
    return false;
  }
}