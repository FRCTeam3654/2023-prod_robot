// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.RobotState;
//import frc.robot.Robot;
//import frc.robot.RobotMap;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualArmCommand extends CommandBase {
  /** Creates a new ManualArmCommand. */
  private int maxLength = 20000;
  private double howFar;
  public ManualArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.telescopingArm);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickX;
    double joystickY;
    
    joystickY = (RobotContainer.oi.operatorStick.getRightY());
    joystickX = (RobotContainer.oi.operatorStick.getRightX());
  
    SmartDashboard.putNumber("JoystickX", joystickX);
    SmartDashboard.putNumber("JoystickY", joystickY);

     
    //RobotContainer.telescopingArm.karenaArcadeDrive(joystickX, joystickY);
   
    if(joystickY > 0.3){
      RobotContainer.telescopingArm.manualJoust(0.3);
      howFar = RobotContainer.telescopingArm.getArmTalonPosition();
      SmartDashboard.putNumber("full extend distance", howFar);
      //if(RobotContainer.telescopingArm.getArmTalonPosition() > maxLength){
       // RobotContainer.telescopingArm.manualJoust(0);
    //  }
    }

    else if(joystickY < -0.3){
      RobotContainer.telescopingArm.manualJoust(-0.3);
      //if(RobotContainer.telescopingArm.getArmTalonPosition() < 0){
        //RobotContainer.telescopingArm.manualJoust(0);
      //}
    }
    else{
      RobotContainer.telescopingArm.manualJoust(0);
      RobotContainer.telescopingArm.getArmTalonPosition();
      
    }
    

    
}

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
