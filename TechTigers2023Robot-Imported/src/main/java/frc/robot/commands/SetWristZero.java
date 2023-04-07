// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.WristMotionMagic;

import edu.wpi.first.wpilibj.Timer;

public class SetWristZero extends CommandBase {
  /** Creates a new SetWristZero. */
  public SetWristZero() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.wrist.zeroSensor();
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
    if(RobotContainer.oi.setWristZeroButton.getAsBoolean() == false){
      return true;
    }
    return false;
  }
}
