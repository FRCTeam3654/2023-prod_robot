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



public class SlidingClimbHooksCommand extends CommandBase {
  private boolean isButtonPressed = false;
  public double slidingClimbTimer = 0;
  private static int climbNumber = 0;
  /** Creates a new SlidingClimbHooksCommand. */
  public SlidingClimbHooksCommand() {
    addRequirements(RobotContainer.slidingClimbHooks);
    // Use addRequirements() here to declare subsystem dependencies.
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
    if ((RobotContainer.oi.slidingClimbButton.get() && !isButtonPressed)){
      isButtonPressed = true;
      if (climbNumber <= 2){
        RobotContainer.slidingClimbHooks.setMotionMagic((1.7)*RobotMap.slidingClimbDistance, 8000, 8000);
      }
      else{
        RobotContainer.slidingClimbHooks.setMotionMagic(RobotMap.slidingClimbDistance, 8000, 8000);
      }
      //RobotContainer.slidingClimbHooks.driveClimbMotors(0.3);
      SmartDashboard.putString("SlidingButtonClicked", "yes");
      slidingClimbTimer = Timer.getFPGATimestamp();
      
    }
    else if ((RobotContainer.oi.slidingClimbReverseButton.get() && !isButtonPressed)){
      isButtonPressed = true;
      if (climbNumber <= 2){
        RobotContainer.slidingClimbHooks.setMotionMagic((-1.7)*RobotMap.slidingClimbDistance, 8000, 8000);
      }
      else {
       RobotContainer.slidingClimbHooks.setMotionMagic((-1)*RobotMap.slidingClimbDistance, 8000, 8000);
      }
      
       //RobotContainer.slidingClimbHooks.driveClimbMotors(0.3);
       SmartDashboard.putString("SlidingReverseButtonClicked", "yes");
       slidingClimbTimer = Timer.getFPGATimestamp();
    
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isButtonPressed = false;
    slidingClimbTimer = 0;
    //RobotContainer.slidingClimbHooks.resetMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(slidingClimbTimer + RobotMap.slidingClimbTimerTimeout < Timer.getFPGATimestamp()) {
      isButtonPressed = false;
      slidingClimbTimer = 0;
      return true;
    } 
    
    else{
      if (isButtonPressed){
        double sensorLeftDistance = RobotContainer.slidingClimbHooks.getClimbHookTalonLeftPosition();
        double sensorRightDistance = RobotContainer.slidingClimbHooks.getClimbHookTalonRightPosition();

        double percentLeftError = 100 * (RobotMap.slidingClimbDistance - sensorLeftDistance)/RobotMap.slidingClimbDistance;
        double percentRightError = 100 * (RobotMap.slidingClimbDistance - sensorRightDistance)/RobotMap.slidingClimbDistance;

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
