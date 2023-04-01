// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;


public class ArmSetPositionsCommand extends CommandBase {
  /** Creates a new TurretSetPositionsCommand. */
  public static int armMoveNumber = 0;
  double armTimer;
  public static boolean isMotionMagicInProgress = false;
  private double armDistance = RobotMap.armFullUpDistance;
  private double armTimerTimeout = 2;
  private boolean isFullPivotPressed = false;
  private boolean isShortPivotUpPressed = false;
  private boolean isShortPivotDownPressed = false;
  private double currentPos;
  private int mode = 0;

  public ArmSetPositionsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.turretSpark);
    addRequirements(RobotContainer.verticalMotionArm);
  }

  public ArmSetPositionsCommand(double distance){
    addRequirements(RobotContainer.verticalMotionArm);
    armDistance = distance;
    armTimerTimeout = 1.5;
    mode = 1;
  }

  public ArmSetPositionsCommand(int new_mode, double new_armTimerTimeout){
    addRequirements(RobotContainer.verticalMotionArm);
    // currently new_mode only = 1 for moving up, 2 for moving down the full distance
    mode = new_mode;
    armTimerTimeout = new_armTimerTimeout;
    System.out.println("ready to lower arm in mode = "+mode);
  }

  public ArmSetPositionsCommand(int new_mode, double new_distance, double new_armTimerTimeout){
    addRequirements(RobotContainer.verticalMotionArm);
    // currently new_mode only = 1 for moving up, 2 for moving down the full distance
    mode = new_mode;
    armDistance = new_distance;
    armTimerTimeout = new_armTimerTimeout;
    System.out.println("ready to lower arm 2 in mode = "+mode);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(RobotContainer.oi.armPivotButton.getAsBoolean() || mode == 1 || mode == 2 || mode == 3){
      isFullPivotPressed = true;
    }

    if(isFullPivotPressed == true ){
      if( mode != 3) {
        armMoveNumber = armMoveNumber + 1;
      }

      if(armMoveNumber %2 == 1 || mode == 1 || mode == 3){ //moves up
        armTimer = Timer.getFPGATimestamp();
        RobotContainer.verticalMotionArm.setMotionMagic(armDistance, 3500, 5000, 0.05);
        //RobotContainer.arm.setMotionMagic(0, 2000, 2000);
        System.out.println("should i be motion magicking up");
        isMotionMagicInProgress = true;
      }

      else if(armMoveNumber %2 != 1 || mode == 2 ){ //moves down
        armTimer = Timer.getFPGATimestamp();
        //RobotContainer.arm.setMotionMagic(RobotMap.armFullUpDistance, 2000, 2000);
        RobotContainer.verticalMotionArm.setMotionMagic(2200, 3000, 3000);     
        
        System.out.println("should i be motion magicking down");
        isMotionMagicInProgress = true;
      }
      else{
        armTimer = Timer.getFPGATimestamp();
        RobotContainer.verticalMotionArm.setMotionMagic(0, 0, 0);
      }
    }

    else if(RobotContainer.oi.armShortPivotDownButton.getAsBoolean()){
      isShortPivotDownPressed = false;
    }

    if(isShortPivotDownPressed == true){
      armTimer = Timer.getFPGATimestamp();
      armTimerTimeout = 1.5;
      //RobotContainer.arm.setMotionMagic(RobotMap.armFullUpDistance, 2000, 2000);
      currentPos = RobotContainer.verticalMotionArm.getArmTalonPosition();
      RobotContainer.verticalMotionArm.setMotionMagic(currentPos - 2000, 2000, 2000);
      System.out.println("should i be motion magicking short down");
      isMotionMagicInProgress = true;
    }
    else if(RobotContainer.oi.armShortPivotUpButton.getAsBoolean()){
      isShortPivotUpPressed = false;
    }

    if(isShortPivotUpPressed == true){
      armTimer = Timer.getFPGATimestamp();
      armTimerTimeout = 1.5;
      //RobotContainer.arm.setMotionMagic(RobotMap.armFullUpDistance, 2000, 2000);
      currentPos = RobotContainer.verticalMotionArm.getArmTalonPosition();
      RobotContainer.verticalMotionArm.setMotionMagic(currentPos + 2000, 2000, 2000);
      System.out.println("should i be motion magicking short up");
      isMotionMagicInProgress = true;
    }
    else{
      //RobotContainer.verticalMotionArm.setMotionMagic(armDistance, 2000, 2000, 0.05);
    }
    
    //RobotContainer.verticalMotionArm.setMotionMagic(RobotMap.armFullUpDistance, 2000, 2000, 0.3);
    //System.out.println("should i be motion magicking up");
    //isMotionMagicInProgress = true;
    //armTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isMotionMagicInProgress = false;
    armDistance = RobotMap.armFullUpDistance;
    armTimerTimeout = 2;
    isFullPivotPressed = false;
    isShortPivotUpPressed = false;
    mode = 0;
    //System.out.println("is the arm command over");
    //RobotContainer.verticalMotionArm.manualVerticalArm(0.2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (armTimer + armTimerTimeout) < Timer.getFPGATimestamp()) {
      // after 3 second, stop command
      isMotionMagicInProgress = false;
      //System.out.println("should i be quitting the arm command");
      return true;
    }
    else {
        double sensorDistance = Math.abs(RobotContainer.wrist.getWristTalonPosition());
        //double percentError = 100 * (RobotMap.wristFullUpDistance - sensorDistance)/RobotMap.wristFullUpDistance;
        double percentError = 100 * (armDistance - sensorDistance)/armDistance;
      
        if (Math.abs(percentError) < 1){
          //if (percentLeftError < 0.9 || percentLeftError < 0 )
          isMotionMagicInProgress = false;
          //System.out.println("should i be quitting the arm command");
          return true;
        }

  }
    return false;
  }
}
