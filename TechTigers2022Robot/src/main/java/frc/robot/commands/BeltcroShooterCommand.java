// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import java.util.concurrent.atomic.AtomicInteger;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;

public class BeltcroShooterCommand extends CommandBase {
  public double beltcroTimer = 0;
  //private AtomicInteger _mode = new AtomicInteger(0); //mode = 0 means button is pressed; mode = 1 means sensor triggers
  private boolean isButtonPressed = false;
  private int mode = 0; //0 is normal, 1 is reverse
  public BeltcroShooterCommand() {
    addRequirements(RobotContainer.beltcro); // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.intake);
  }
  public BeltcroShooterCommand(int new_mode) {
    addRequirements(RobotContainer.beltcro); // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.intake);
    mode = new_mode;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    beltcroTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if ((RobotContainer.oi.beltcroShooterButton.get() && !isButtonPressed)){
      //isButtonPressed = true;
    //}
    //if (isButtonPressed==true){
    if (RobotContainer.oi.beltcroShooterButton.get()){
      mode = 0;
    }
    else if (RobotContainer.oi.beltcroReverseButton.get()){
      mode = 1;
    }
    if (mode == 0){
      RobotContainer.beltcro.beltcroMove(RobotMap.beltcroSpeed);
      RobotContainer.intake.intakeWheels(RobotMap.intakeSpeedIn);
    }
    else if (mode == 1){
      RobotContainer.beltcro.beltcroMove((-1)*RobotMap.beltcroSpeed);
      RobotContainer.intake.intakeWheels((-1)*RobotMap.intakeSpeedIn);
    }
    else if (mode == 2){
      RobotContainer.beltcro.beltcroMove((-1)*RobotMap.beltcroSpeed);
      RobotContainer.intake.intakeWheels(RobotMap.intakeSpeedOut);
    }
   // }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.beltcro.beltcroMove(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(beltcroTimer + RobotMap.beltcroTimerTimeout < Timer.getFPGATimestamp()) {
      RobotContainer.beltcro.beltcroMove(0);
       return true;
  }
    return false;
  }
}
