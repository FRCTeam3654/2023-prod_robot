// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoArmJoustCommand extends CommandBase {

  public double joustTimer = 0;
  private int mode = 0; //0 is normal, 1 is auto out, 2 is auto in


  /** Creates a new AutoArmJoustCommand. */
  public AutoArmJoustCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.telescopingArm);
  }

  public AutoArmJoustCommand(int new_mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    mode = new_mode;
    addRequirements(RobotContainer.telescopingArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(mode == 1){ //moves down
      RobotContainer.telescopingArm.setMotionMagic(RobotMap.joustExtendDistance, 8000, 8000); //maximum speed is 22000
      joustTimer = Timer.getFPGATimestamp();
    }
    else  if(mode == 2){ //moves up
      RobotContainer.telescopingArm.setMotionMagic(0, 8000, 8000); //8000 8000
      joustTimer = Timer.getFPGATimestamp();
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

    if(joustTimer + RobotMap.joustTimerTimeout < Timer.getFPGATimestamp()) {
      joustTimer = 0;   
      //RobotContainer.telescopingArm.setMotionMagic(RobotContainer.telescopingArm.getArmTalonPosition(), 8000, 8000, 0);
      return true;
    } 
    
    else{
      if( mode ==1) {
          double sensorDistance = RobotContainer.telescopingArm.getArmTalonPosition();

          double percentError = 100 * (RobotMap.joustExtendDistance - sensorDistance)/RobotMap.joustExtendDistance;
        
          SmartDashboard.putNumber("percentErrorjoustLeft", percentError);
          if ((percentError < 1 || percentError < 0 )){
            return true;
          }
        } 
    }

    return false;
  }
}
