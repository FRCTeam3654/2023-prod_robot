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

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  NetworkTableEntry isRedAlliance;
  private boolean isBeltcroMoving = false;
  private boolean isEjectingBall = false;
  public double intakeEjectTimer = 0;
  public double beltcroIntakeTimer = 0;
  public int mode = 0; //0 is default, 1 is auto, 2 is intake override
  public IntakeCommand() {
    addRequirements(RobotContainer.intake);
    //addRequirements(RobotContainer.beltcro);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public IntakeCommand(int new_mode) {
    addRequirements(RobotContainer.intake);
    //addRequirements(RobotContainer.beltcro);
    mode = new_mode;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public IntakeCommand(double timeout) {
    addRequirements(RobotContainer.intake);
    //addRequirements(RobotContainer.beltcro);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

      //get a reference to the subtable called "datatable"
      NetworkTable fmsInfo = inst.getTable("FMSInfo");

      //get a reference to key in "datatable" called "Y"
      isRedAlliance = fmsInfo.getEntry("IsRedAlliance");
    RobotContainer.beltcro.beltcroMove(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int colorNumber;
    boolean allianceColor; //if allianceColor is true, we are RED team
    allianceColor = isRedAlliance.getBoolean(false);
    colorNumber = RobotContainer.intake.getRainbow();
    if (RobotContainer.oi.intakeOverrideButton.get()){
      mode = 2;
    }
    if (mode == 1){
      RobotContainer.intake.intakeWheels(RobotMap.intakeSpeedOut);
      //RobotContainer.beltcro.beltcroMove((-1)*RobotMap.beltcroSpeed);
    }
    if (mode == 2){
      RobotContainer.intake.intakeWheels(RobotMap.intakeSpeedIn);
      if (!RobotContainer.oi.intakeOverrideButton.get()){
        mode = 0;
      }
    }
    else
    if (mode == 0){
    if (allianceColor){
      if (colorNumber == 1) {
        //if it sees a blue ball and we're on the red team, it reverses intake wheels
        if (!isEjectingBall){
          isEjectingBall = true;
          intakeEjectTimer = Timer.getFPGATimestamp();
          if (!BallShooterCommand.isShooterInProgress){
          RobotContainer.beltcro.beltcroMove(0);
          }
        }
      }
      if (colorNumber == 2) {
        if (!isBeltcroMoving){
          isBeltcroMoving = true;
          beltcroIntakeTimer = Timer.getFPGATimestamp();
        }
      }
      if (colorNumber == 0) { 
      //is not a blue ball and we're on the red team, it keeps intake going the same way
      }
    }
    else {
      if (colorNumber == 2) {
        //if it sees a red ball and we're on the blue team, it spits it away
        if (!isEjectingBall){
          isEjectingBall = true;
          intakeEjectTimer = Timer.getFPGATimestamp();
          if (!BallShooterCommand.isShooterInProgress){
            RobotContainer.beltcro.beltcroMove(0);
            }
          }
      }
      if (colorNumber == 1) {
        if (!isBeltcroMoving){
          isBeltcroMoving = true;
          beltcroIntakeTimer = Timer.getFPGATimestamp();
        }
      }
      if (colorNumber == 0) {
        // if it isn't a red ball and we're on the blue team, it runs intake
        } 
      }
    if (beltcroIntakeTimer + RobotMap.beltcroIntakeTimerTimeout < Timer.getFPGATimestamp()) {
      isBeltcroMoving = false;
      }
      if (isBeltcroMoving){
        if (mode != 1){
        if(RobotContainer.beltcro.storageSensor1() < 1200) {
            RobotContainer.beltcro.beltcroMove(RobotMap.beltcroSpeed); 
          }
        else{
          if (!BallShooterCommand.isShooterInProgress){
            RobotContainer.beltcro.beltcroMove(0);
            }
        }
        }
    
      }
      else {
        if (!BallShooterCommand.isShooterInProgress){
          RobotContainer.beltcro.beltcroMove(0);
          }
        }
    if (intakeEjectTimer + RobotMap.intakeEjectTimerTimeout < Timer.getFPGATimestamp()) {
      isEjectingBall = false;
      }
      if (mode == 1){
        isEjectingBall = true;
      }
      if (isEjectingBall){
        RobotContainer.intake.intakeWheels(RobotMap.intakeSpeedOut);
        if (!BallShooterCommand.isShooterInProgress){
          RobotContainer.beltcro.beltcroMove(0);
          }
        if (mode != 1){
        //RobotContainer.beltcro.beltcroMove(RobotMap.beltcroSpeed);
        }
      }
      else {
        RobotContainer.intake.intakeWheels(RobotMap.intakeSpeedIn);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.intakeWheels(0);
    if (!BallShooterCommand.isShooterInProgress){
      RobotContainer.beltcro.beltcroMove(0);
      }
    mode = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
