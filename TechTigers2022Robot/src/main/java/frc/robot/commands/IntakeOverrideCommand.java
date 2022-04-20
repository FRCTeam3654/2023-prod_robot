// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
//import java.util.concurrent.atomic.AtomicInteger;
//import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;


public class IntakeOverrideCommand extends CommandBase {
  private boolean IsButtonPressed=true;
  private boolean WasButtonNotPressed=false;
  private boolean isBeltcroMoving = false;

  /** Creates a new IntakeOverrideCommand. */
  public IntakeOverrideCommand() {
    addRequirements(RobotContainer.intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IsButtonPressed=true;
    WasButtonNotPressed=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake.intakeWheels(RobotMap.intakeSpeedIn);
    IsButtonPressed=RobotContainer.oi.intakeOverrideButton.get();
    isBeltcroMoving = true;
      if (!IsButtonPressed) {
        WasButtonNotPressed=true;
      }
      if (isBeltcroMoving){
        if(RobotContainer.beltcro.storageSensor1() < 1000) {
            RobotContainer.beltcro.beltcroMove(RobotMap.beltcroSpeed); 
          }
        else{
          if (!BallShooterCommand.isShooterInProgress){
            RobotContainer.beltcro.beltcroMove(0);
            }
        }
        }
      //SmartDashboard.putBoolean("IsButtonPressed", IsButtonPressed);
      //SmartDashboard.putBoolean("WasButtonNotPressed", WasButtonNotPressed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (IsButtonPressed && WasButtonNotPressed){
      return true;
    }
    return false;
  }
}
