// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
//import java.util.concurrent.atomic.AtomicInteger;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;

public class AutoIntakeWheelsCommand extends CommandBase {
  /** Creates a new IntakeWheelsCommand. */

  public double intakeTimer = 0;
  private double intakeTimeout = 2;
  public boolean isHolding = false;
  public int mode = 0; //0 is default slow move, 1 is full in, 2 is full out
  private double wheelIntakePower = RobotMap.intakeSpeed;


  public AutoIntakeWheelsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wheelIntake);
  }

  public AutoIntakeWheelsCommand(int mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wheelIntake);
    this.mode = mode;
  }

  public AutoIntakeWheelsCommand(int mode, double new_intakepower) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wheelIntake);
    this.mode = mode;
    this.wheelIntakePower = new_intakepower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mode == 1){
      RobotContainer.wheelIntake.intakeWheels(RobotMap.intakeSpeed);
      //intakeTimer = Timer.getFPGATimestamp();
    }

    else if(mode == 2){
      RobotContainer.wheelIntake.intakeWheels(-1 * RobotMap.intakeSpeed);
      //intakeTimer = Timer.getFPGATimestamp();
    }

    else if(mode == 3){
      RobotContainer.wheelIntake.intakeWheels(-1.4 * RobotMap.intakeSpeed);
      //intakeTimer = Timer.getFPGATimestamp();
    }

    else if(mode == 0){
      RobotContainer.wheelIntake.intakeWheels(0.25 * RobotMap.intakeSpeed);
      //intakeTimer = Timer.getFPGATimestamp();
    }

    else if(mode == 4){
      RobotContainer.wheelIntake.intakeWheels(0.25 * RobotMap.intakeSpeed);
      //intakeTimer = Timer.getFPGATimestamp();
      intakeTimeout = 0.5;
    }

    else if(mode == 5){
      RobotContainer.wheelIntake.intakeWheels(-0.8 * RobotMap.intakeSpeed);
      //intakeTimer = Timer.getFPGATimestamp();
      intakeTimeout = 0.8;
    }

    else if(mode == 6){
      RobotContainer.wheelIntake.intakeWheels(-1 * RobotMap.intakeSpeed);
      //intakeTimer = Timer.getFPGATimestamp();
    }

    else if(mode == 7){
      RobotContainer.wheelIntake.intakeWheels(RobotMap.intakeSpeed);
      //intakeTimer = Timer.getFPGATimestamp();
      intakeTimeout = 0.5;
    }

    if(RobotContainer.oi.operatorStick.getRightTriggerAxis() > 0.4){
      RobotContainer.wheelIntake.intakeWheels(0);
    }

    //RobotContainer.wheelIntake.intakeWheels(0.35 * RobotMap.intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //mode = 0;
  RobotContainer.wheelIntake.intakeWheels(0);
  intakeTimeout = 2;
  //intakeTimer = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((intakeTimer + intakeTimeout < Timer.getFPGATimestamp())){
      return true;
    }
    return false;
  }
}
