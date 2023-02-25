// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;

public class ManualVerticalArmCommand extends CommandBase {
  /** Creates a new ManualVerticalArmCommand. */

  private boolean isLockButtonPressed; 


  public ManualVerticalArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.verticalMotionArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double joystickX;
    double joystickY;
    joystickY = (RobotContainer.oi.operatorStick.getRightY());

    
    joystickY = handleDeadband(joystickY, RobotMap.joystickDeadBand);

    
    //joystickX = (RobotContainer.oi.operatorStick.getRightX());
  
    if(RobotContainer.oi.armLockButton.getAsBoolean()){
      isLockButtonPressed = true;
    }

    if(isLockButtonPressed = true){
      RobotContainer.verticalMotionArm.manualVerticalArm(0);
    }

    if(RobotContainer.oi.operatorStick.getRightY() > RobotMap.joystickDeadBand){
      isLockButtonPressed = false;
    }
    
    if(isLockButtonPressed == false){
      RobotContainer.verticalMotionArm.manualVerticalArm(joystickY);
    }
    

    else{
      RobotContainer.verticalMotionArm.manualVerticalArm(joystickY);
   }
  }

  public double handleDeadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.verticalMotionArm.manualVerticalArm(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
