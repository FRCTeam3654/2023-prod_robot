// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;


public class ManualTurretTurningCommand extends CommandBase {
  /** Creates a new TurretTurningCommand. */
  private int desiredTurnAngle = 0;
  private boolean turn90Flag = false;
  private double turnEndpointAngle = 0;//vinnies desired finish endpoint angle 
  private double vinniesError = 2;//greater than 1 so it doesn't get triggered until the error is correct
  private double startTime90degree = 0;

  public ManualTurretTurningCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSpark);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.turretSpark.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //RobotContainer.oi.turretButton.
    if(RobotContainer.oi.turretRightPOV.getAsBoolean()){
      RobotContainer.turretSpark.manualTurretControl(3);
    }

    if(RobotContainer.oi.turretLeftPOV.getAsBoolean()){
      RobotContainer.turretSpark.manualTurretControl(-3);
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turretSpark.manualTurretControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (RobotContainer.turret.turretTickCount() >= RobotMap.turretTickAmount){
      //RobotContainer.turret.zeroSensor();
      //return true;
    //}
    return false;
  }
}
