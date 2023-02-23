// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PneumaticsGrabbingCommand extends CommandBase {
  /** Creates a new PneumaticsGrabbingCommand. */
  public PneumaticsGrabbingCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pneumaticGrab);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.oi.operatorStick.getRightTriggerAxis() > 4){
      RobotContainer.pneumaticGrab.practiceSolenoid(true);
    }

    else if(RobotContainer.oi.operatorStick.getLeftTriggerAxis() > 4){
      RobotContainer.pneumaticGrab.practiceSolenoid(false);
    }

    else{
      RobotContainer.pneumaticGrab.practiceSolenoid(false);

    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if (RobotContainer.oi.pneumaticGrabButton.getAsBoolean() == false) {
      RobotContainer.pneumaticGrab.practiceSolenoid(false);
      return true;
    }
    */
    return false;
  }
}
