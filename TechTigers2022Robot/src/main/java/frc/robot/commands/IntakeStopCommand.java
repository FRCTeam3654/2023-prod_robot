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

public class IntakeStopCommand extends CommandBase {
  /** Creates a new IntakeStopCommand. */
  private boolean IsButtonPressed=true;
  private boolean WasButtonNotPressed=false;
  public IntakeStopCommand() {
    addRequirements(RobotContainer.intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake.intakeWheels(0);
    IsButtonPressed=RobotContainer.oi.intakeStopButton.getAsBoolean();
      if (!IsButtonPressed) {
        WasButtonNotPressed=true;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.intakeWheels(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (IsButtonPressed && WasButtonNotPressed){
      return true;
    }
    return false;
  }
}
