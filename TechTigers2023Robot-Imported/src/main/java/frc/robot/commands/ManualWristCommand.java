// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.WristMotionMagic;

import edu.wpi.first.wpilibj.Timer;


public class ManualWristCommand extends CommandBase {
  /** Creates a new WristCommand. */
  private double wristHoldingPower = 0.12;
  private double wristPos;

  public ManualWristCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.oi.wristUpPOV.getAsBoolean() == true){
      RobotContainer.wrist.manualwrist(0.15);
      wristPos = RobotContainer.wrist.getWristTalonPosition();
    }
    else if(RobotContainer.oi.wristDownPOV.getAsBoolean() == true){
      RobotContainer.wrist.manualwrist(-0.15);
      wristPos = RobotContainer.wrist.getWristTalonPosition();
    }

    else{
      /*if(WristMotionMagic.wristMoveNumber %2 == 1 && WristMotionMagic.isMotionMagicInProgress == false){
        //need to figure out the angles times the cosine angles
        double currentSensorReading = RobotContainer.wrist.getWristTalonPosition();
        double theta = ((Math.abs(RobotMap.wristFullUpDistance) - Math.abs(currentSensorReading))/Math.abs(RobotMap.wristFullUpDistance)) * 90;
        RobotContainer.wrist.manualwrist(wristHoldingPower * Math.cos(Math.toRadians(theta)));
      }
      else if(WristMotionMagic.wristMoveNumber %2 == 0 && WristMotionMagic.isMotionMagicInProgress == false){
        RobotContainer.wrist.manualwrist(0);
      }
      //RobotContainer.wrist.
      //System.out.println("Should i be staying still");
      */
      RobotContainer.wrist.setMotionMagic(0, 0, 0);
    }

  //need to add an if statement for if it rotates x amount of revolutions it stops
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.wrist.setMotionMagic(wristPos, 5000, 5000);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.oi.wristUpPOV.getAsBoolean() == false && RobotContainer.oi.wristDownPOV.getAsBoolean() == false){
      return true;
    }

    return false;
  }
}
