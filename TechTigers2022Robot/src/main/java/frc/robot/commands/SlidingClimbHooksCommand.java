// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;



public class SlidingClimbHooksCommand extends CommandBase {
  private boolean isButtonPressed = false;
  public double slidingClimbTimer = 0;
  public static int climbNumber = 0;//IN AUTONOMOUS WE WILL PULL THIS DOWN FIRST SO AFTER TESTING IS DONE MAKE THIS 1 NOT 0
  private static double sliderCurrentPosition  = 0;
  private double distanceToBeTraveled = 0;
  private double targetedDistance = 0;
  private int mode = 0; // mode 0 is normal, mode 1 is auto, and mode 2 is short climb distance, 3 is traversal down


  /** Creates a new SlidingClimbHooksCommand. */
  public SlidingClimbHooksCommand() {
    addRequirements(RobotContainer.slidingClimbHooks);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public SlidingClimbHooksCommand(int new_mode) {
    addRequirements(RobotContainer.slidingClimbHooks);
    mode = new_mode;
    if (mode == 1){
      RobotMap.slidingClimbTimerTimeout = 6;
    }
    else{
      RobotMap.slidingClimbTimerTimeout = 15;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  public SlidingClimbHooksCommand(double distance_to_travel) {
    addRequirements(RobotContainer.slidingClimbHooks);
    // Use addRequirements() here to declare subsystem dependencies.
	  targetedDistance = distance_to_travel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.slidingClimbHooks.zeroSensors();
    climbNumber = climbNumber + 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (((RobotContainer.oi.slidingClimbButton.get() || mode == 1) && !isButtonPressed)){
      isButtonPressed = true;
      if (climbNumber <= 0){
        distanceToBeTraveled = RobotMap.slidingClimbFullDistance;
        RobotContainer.slidingClimbHooks.setMotionMagic(RobotMap.slidingClimbFullDistance, 8000, 8000);
      }
      else{
        distanceToBeTraveled = RobotMap.slidingClimbDistance;
        RobotContainer.slidingClimbHooks.setMotionMagic(RobotMap.slidingClimbDistance, 4000, 8000);
      }
      //RobotContainer.slidingClimbHooks.driveClimbMotors(0.3);
      SmartDashboard.putString("SlidingButtonClicked", "yes");
      slidingClimbTimer = Timer.getFPGATimestamp();
      
    }
    else if ((RobotContainer.oi.slidingClimbReverseButton.get() && !isButtonPressed)){
      isButtonPressed = true;
      if (climbNumber <= 0){
        distanceToBeTraveled = (-1)*RobotMap.slidingClimbFullDistance;
        RobotContainer.slidingClimbHooks.setMotionMagic((-1)*RobotMap.slidingClimbFullDistance, 8000, 8000);
      }
      else {
        distanceToBeTraveled = (-1)*RobotMap.slidingClimbDistance; 
        // TODO:  adjust above distance by sliderCurrentPosition (instead of above, should be (-1)*sliderCurrentPosition
        // distanceToBeTraveled = (-1) * sliderCurrentPosition ;// double check ??
        
        RobotContainer.slidingClimbHooks.setMotionMagic((-1)*RobotMap.slidingClimbDistance + RobotMap.slidingShortClimbDistance, 4000, 8000);
        //RobotContainer.slidingClimbHooks.setMotionMagic((-1)*RobotMap.slidingClimbDistance, 4000, 8000);

      }
      
       //RobotContainer.slidingClimbHooks.driveClimbMotors(0.3);
       SmartDashboard.putString("SlidingReverseButtonClicked", "yes");
       slidingClimbTimer = Timer.getFPGATimestamp();  
    }
    
    else if ((RobotContainer.oi.slidingShortClimbButton.get() && !isButtonPressed)){
      isButtonPressed = true;

      mode = 2;
      
      distanceToBeTraveled = targetedDistance;
      RobotContainer.slidingClimbHooks.setMotionMagic(distanceToBeTraveled, 4000, 8000);
 
      SmartDashboard.putString("SlidingShortClimbButtonClicked", "yes");
      slidingClimbTimer = Timer.getFPGATimestamp();
    }
    else if ((RobotContainer.oi.slidingShortClimbReverseButton.get() && !isButtonPressed)){
      isButtonPressed = true;
      
      mode = 2;

      distanceToBeTraveled = (-1) * targetedDistance;
      RobotContainer.slidingClimbHooks.setMotionMagic(distanceToBeTraveled, 4000, 8000);
 
      SmartDashboard.putString("SlidingShortClimbReverseButtonClicked", "yes");
      slidingClimbTimer = Timer.getFPGATimestamp();
    }
    else if ((RobotContainer.oi.slidingTraversalDownButton.get() && !isButtonPressed)){
      isButtonPressed = true;

      mode = 3;
      
      distanceToBeTraveled = RobotMap.slidingTravesalDownDistance;
      RobotContainer.slidingClimbHooks.setMotionMagic(distanceToBeTraveled, 4000, 8000);
 
      SmartDashboard.putString("SlidingTraversalDownButtonClicked", "yes");
      slidingClimbTimer = Timer.getFPGATimestamp();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isButtonPressed = false;
    slidingClimbTimer = 0;
    mode = 0;
    VerticalClimbHoldCommand.cancelLock = false;
    sliderCurrentPosition = sliderCurrentPosition + distanceToBeTraveled; // where the slider is currently, 
    SmartDashboard.getNumber("sliderCurrentPosition", sliderCurrentPosition);
    //RobotContainer.slidingClimbHooks.resetMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // tell VerticalCombCommand that it is moving down after 1 second  and less than 3 seconds
    if( (slidingClimbTimer + 1.0) < Timer.getFPGATimestamp() &&  (slidingClimbTimer + 3.0) > Timer.getFPGATimestamp() ) {
        if (climbNumber > 2 && distanceToBeTraveled > 0 ){
           VerticalClimbHoldCommand.cancelLock = true; // tell VerticalClimb to unlock if it is locked
        }
    }
    else if( (slidingClimbTimer + 3.0) < Timer.getFPGATimestamp()) {
      VerticalClimbHoldCommand.cancelLock = false; // after 3 second, stop notifying, optional
    }

    double newTimeout = RobotMap.slidingClimbTimerTimeout;
    if (mode == 2){
      newTimeout = 1.5;
    }
    if (mode == 3){
      newTimeout = 8;
    }

    if(slidingClimbTimer + newTimeout < Timer.getFPGATimestamp()) {
      isButtonPressed = false;
      slidingClimbTimer = 0;
      return true;
    } 
    else {
      if (isButtonPressed){
        double sensorLeftDistance = RobotContainer.slidingClimbHooks.getClimbHookTalonLeftPosition();
        double sensorRightDistance = RobotContainer.slidingClimbHooks.getClimbHookTalonRightPosition();

        // TODO:   uncomment the following, comment out line 126, 127
        //double percentLeftError = 100 * (Math.abs(distanceToBeTraveled) - Math.abs(sensorLeftDistance))/Math.abs(distanceToBeTraveled);
        //double percentRightError = 100 * (Math.abs(distanceToBeTraveled) - Math.abs(sensorRightDistance))/Math.abs(distanceToBeTraveled);


        double percentLeftError = 100 * (RobotMap.slidingClimbDistance - sensorLeftDistance)/RobotMap.slidingClimbDistance;
        double percentRightError = 100 * (RobotMap.slidingClimbDistance - sensorRightDistance)/RobotMap.slidingClimbDistance;
       
       //if (percentLeftError < 0.7 && percentRightError < 0.7 && slidingClimbTimer > 2 && (RobotContainer.oi.slidingClimbButton.get() && isButtonPressed)){
          //RobotContainer.verticalClimbArms.resetMotors();
        //}


        SmartDashboard.putNumber("percentErrorClimbLeft", percentLeftError);
      //even though it is desired to achieve error < 1%, it depends on PID tuning, sometimes it is always achieable
          if ((percentLeftError < 0.3 || percentLeftError < 0 ) && (percentRightError < 0.3 || percentRightError < 0 ))
          //if (percentLeftError < 0.9 || percentLeftError < 0 )
          return true;
        }
        }
      
        return false;
  }
}
