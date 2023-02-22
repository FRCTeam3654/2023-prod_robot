// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;


public class ManualWristCommand extends CommandBase {
  /** Creates a new WristCommand. */
  
  public ManualWristCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  double joystickY;
  joystickY = (RobotContainer.oi.driverStick.getLeftY() * -1);
  joystickY = handleDeadband(joystickY, RobotMap.joystickDeadBand);

    if(RobotContainer.oi.wristUpPOV.getAsBoolean() == true || RobotContainer.oi.wristDownPOV.getAsBoolean()){
      RobotContainer.wrist.manualwrist(joystickY * 0.2);
    }
    
    else{
      RobotContainer.wrist.manualwrist(0);
    }

  //need to add an if statement for if it rotates x amount of revolutions it stops
  }


  public double handleDeadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
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
