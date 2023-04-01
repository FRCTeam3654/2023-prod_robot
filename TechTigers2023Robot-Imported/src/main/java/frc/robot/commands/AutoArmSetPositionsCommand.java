// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;


public class AutoArmSetPositionsCommand extends CommandBase {
  /** Creates a new TurretSetPositionsCommand. */
  //public static int armMoveNumber = 0;
  double armTimer;
  public static boolean isMotionMagicInProgress = false;
  private double armDistance = RobotMap.armFullUpDistance;
  private double armShortDistance = 2200;
  private double armTimerTimeout = 2;
  private int mode = 0;

  public AutoArmSetPositionsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.turretSpark);
    addRequirements(RobotContainer.verticalMotionArm);
  }

  public AutoArmSetPositionsCommand(double distance){
    addRequirements(RobotContainer.verticalMotionArm);
    armDistance = distance;
    armTimerTimeout = 1.5;
    mode = 1;
  }

  public AutoArmSetPositionsCommand(int new_mode, double new_armTimerTimeout){
    addRequirements(RobotContainer.verticalMotionArm);
    // currently new_mode only = 1 for moving up, 2 for moving down the full distance
    mode = new_mode;
    armTimerTimeout = new_armTimerTimeout;
    System.out.println("ready to lower arm in mode = "+mode);
  }

  public AutoArmSetPositionsCommand(int new_mode, double new_distance, double new_armTimerTimeout){
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
    
      if( mode == 1 ){ //moves up
        armTimer = Timer.getFPGATimestamp();
        RobotContainer.verticalMotionArm.setMotionMagic(armDistance, 3500, 5000, 0.05);

        System.out.println("auto arm up");
        isMotionMagicInProgress = true;
      }

      else if( mode == 2 ){ //moves down
        armTimer = Timer.getFPGATimestamp();
        RobotContainer.verticalMotionArm.setMotionMagic(2200, 3000, 3000);     
        
        System.out.println("auto arm down");
        isMotionMagicInProgress = true;
      }
      else if( mode == 3 ){ //moves up a little 2200
        armTimer = Timer.getFPGATimestamp();
        // armShortDistance = 2200;
        RobotContainer.verticalMotionArm.setMotionMagic(armShortDistance, 3500, 5000, 0.05);

        System.out.println("auto arm up a little");
        isMotionMagicInProgress = true;
      }
    
    
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
    //mode = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (armTimer + armTimerTimeout) < Timer.getFPGATimestamp()) {
      // after 2 second, stop command
      isMotionMagicInProgress = false;
      return true;
    }
    else {
       // double sensorDistance = Math.abs(RobotContainer.wrist.getWristTalonPosition());
      if( mode == 1 ) {
            double sensorDistance = Math.abs(RobotContainer.verticalMotionArm.getArmTalonPosition());
            //double percentError = 100 * (RobotMap.wristFullUpDistance - sensorDistance)/RobotMap.wristFullUpDistance;
            double percentError = 100 * (armDistance - sensorDistance)/armDistance;
          
            if (Math.abs(percentError) < 1){
              //if (percentLeftError < 0.9 || percentLeftError < 0 )
              isMotionMagicInProgress = false;
              //System.out.println("should i be quitting the arm command");
              //return true;
            }
      }
      else if( mode == 2) {
        double sensorDistance = Math.abs(RobotContainer.verticalMotionArm.getArmTalonPosition());
        if ((sensorDistance - 2200) < 10) {
         //  return true;
        }      
      }
      else if( mode == 3) {
           double sensorDistance = Math.abs(RobotContainer.verticalMotionArm.getArmTalonPosition());
           
            double percentError = 100 * (armShortDistance - sensorDistance)/armShortDistance;
          
            if (Math.abs(percentError) < 1){
              //if (percentLeftError < 0.9 || percentLeftError < 0 )
              isMotionMagicInProgress = false;
              //System.out.println("should i be quitting the arm command");
              //return true;
            }
      }

  }
    return false;
  }
}
