// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;


public class ArmSetPositionsCommand extends CommandBase {
  /** Creates a new TurretSetPositionsCommand. */
  public static int armMoveNumber = 0;
  double armTimer;
  public static boolean isMotionMagicInProgress = false;

  public ArmSetPositionsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.turretSpark);
    addRequirements(RobotContainer.verticalMotionArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armMoveNumber = armMoveNumber + 1;

    if(armMoveNumber %2 == 1){ //moves up
      armTimer = Timer.getFPGATimestamp();
      RobotContainer.verticalMotionArm.setMotionMagic(RobotMap.armFullUpDistance, 2000, 2000);
      //RobotContainer.arm.setMotionMagic(0, 2000, 2000);
      System.out.println("should i be motion magicking up");
      isMotionMagicInProgress = true;
    }

    if(armMoveNumber %2 != 1){ //moves down
      armTimer = Timer.getFPGATimestamp();
      //RobotContainer.arm.setMotionMagic(RobotMap.armFullUpDistance, 2000, 2000);
      RobotContainer.verticalMotionArm.setMotionMagic(0, 2000, 2000);
      System.out.println("should i be motion magicking down");
      isMotionMagicInProgress = true;
    }
    else{
      RobotContainer.verticalMotionArm.setMotionMagic(0, 0, 0);
      armTimer = Timer.getFPGATimestamp();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isMotionMagicInProgress = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (armTimer + 3.0) < Timer.getFPGATimestamp()) {
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
