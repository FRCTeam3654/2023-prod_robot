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
  private static int wristMoveNumber = 0;
  double wristTimer;

  public WristMotionMagic() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.wrist.zeroSensor();
    wristMoveNumber = wristMoveNumber + 1;

    if(wristMoveNumber %2 == 1){
      wristTimer = Timer.getFPGATimestamp();
      RobotContainer.wrist.setMotionMagic(-1 * RobotMap.wristFullUpDistance, 2000, 2000);
      System.out.println("should i be motion magicking down");
    }

    if(wristMoveNumber %2 != 1){
      wristTimer = Timer.getFPGATimestamp();
      RobotContainer.wrist.setMotionMagic(RobotMap.wristFullUpDistance, 2000, 2000);
      System.out.println("should i be motion magicking up");
    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (wristTimer + 3.0) < Timer.getFPGATimestamp()) {
      // after 3 second, stop command
      return true;
    }
    else {
        double sensorDistance = Math.abs(RobotContainer.wrist.getWristTalonPosition());
        double percentError = 100 * (RobotMap.wristFullUpDistance - sensorDistance)/RobotMap.wristFullUpDistance;

        if (Math.abs(percentError) < 1){
        //if (percentLeftError < 0.9 || percentLeftError < 0 )
        return true;
        }

  }
    return false;
  }
}
