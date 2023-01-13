// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class TurretTurningCommand extends CommandBase {
  /** Creates a new TurretTurningCommand. */
  public TurretTurningCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.turret.zeroSensor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.turret.turretTurning(RobotMap.turretTickAmount);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.manualTurret(0.0);
    RobotContainer.turret.zeroSensor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.turret.turretTickCount() >= RobotMap.turretTickAmount){
      RobotContainer.turret.zeroSensor();
      return true;
    }
    return false;
  }
}
