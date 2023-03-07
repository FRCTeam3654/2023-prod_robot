// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmJoustCommand extends CommandBase {
  /** Creates a new ArmJoustCommand. */
  private boolean isButtonPressed = false;
  public double joustTimer = 0;
  private int mode = 0; //0 is normal, 1 is auto out, 2 is auto in

  public ArmJoustCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.telescopingArm);
  }

  public ArmJoustCommand(int new_mode){
    addRequirements(RobotContainer.telescopingArm);
    mode = new_mode;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((RobotContainer.oi.armFullOutButton.getAsBoolean() && !isButtonPressed) || mode == 1){ //extends it
      isButtonPressed = true;
      RobotContainer.telescopingArm.setMotionMagic(RobotMap.joustExtendDistance, 8000, 8000); //maximum speed is 22000
      //RobotContainer.slidingClimbHooks.driveClimbMotors(0.3);
      SmartDashboard.putString("verticalUpButtonClicked", "yes");
      joustTimer = Timer.getFPGATimestamp();
    }
    else if ((RobotContainer.oi.armFullBackButton.getAsBoolean() && !isButtonPressed) || mode == 2){//pulls it back
      isButtonPressed = true;
      RobotContainer.telescopingArm.setMotionMagic(0, 8000, 8000); //8000 8000

    
    
      SmartDashboard.putString("JoustButtonClicked", "yes");
      joustTimer = Timer.getFPGATimestamp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isButtonPressed = false;
    joustTimer = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(joustTimer + RobotMap.joustTimerTimeout < Timer.getFPGATimestamp()) {
      isButtonPressed = false;
      joustTimer = 0;
      //RobotContainer.telescopingArm.setMotionMagic(RobotContainer.telescopingArm.getjoustLeftTalonPosition(), 8000, 8000, 0.1);
      RobotContainer.telescopingArm.setMotionMagic(RobotContainer.telescopingArm.getArmTalonPosition(), 8000, 8000, 0);
      return true;
    } 
    
    else{
      if (isButtonPressed){
        double sensorDistance = RobotContainer.telescopingArm.getArmTalonPosition();

        double percentError = 100 * (RobotMap.joustExtendDistance - sensorDistance)/RobotMap.joustExtendDistance;
        
        SmartDashboard.putNumber("percentErrorjoustLeft", percentError);
          if ((percentError < 0.5 || percentError < 0 )){
            return true;
          }
        }
      }
    return false;
  }
}
