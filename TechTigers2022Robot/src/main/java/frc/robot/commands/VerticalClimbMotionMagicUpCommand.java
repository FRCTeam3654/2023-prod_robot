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


public class VerticalClimbMotionMagicUpCommand extends CommandBase {
  /** Creates a new VerticalClimbMotionMagicCommand. */
  private boolean isButtonPressed = false;
  public double verticalClimbTimer = 0;

  public VerticalClimbMotionMagicUpCommand() {
    addRequirements(RobotContainer.verticalClimbArms);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.verticalClimbArms.zeroSensors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((RobotContainer.oi.verticalClimbUpButton.get() && !isButtonPressed)){
      isButtonPressed = true;
      RobotContainer.verticalClimbArms.setMotionMagic(RobotMap.verticalClimbDistance, 8000, 8000); //maximum speed is 22000
      //RobotContainer.slidingClimbHooks.driveClimbMotors(0.3);
      SmartDashboard.putString("verticalUpButtonClicked", "yes");
      verticalClimbTimer = Timer.getFPGATimestamp();
    }
    else if ((RobotContainer.oi.verticalClimbDownButton.get() && !isButtonPressed)){
      isButtonPressed = true;
      RobotContainer.verticalClimbArms.setMotionMagic((-1)*RobotMap.verticalClimbDistance, 8000, 8000); //8000 8000

      // TODO:  test ArbitraryFeedForward,  is it positive or negative for reverse climb ?
      //RobotContainer.verticalClimbArms.setMotionMagic((-1)*RobotMap.verticalClimbDistance, 8000, 8000, 0.07);
    
    
      SmartDashboard.putString("verticalDownButtonClicked", "yes");
      verticalClimbTimer = Timer.getFPGATimestamp();
    }

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isButtonPressed = false;
    verticalClimbTimer = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(verticalClimbTimer + RobotMap.verticalClimbTimerTimeout < Timer.getFPGATimestamp()) {
      isButtonPressed = false;
      verticalClimbTimer = 0;
      //RobotContainer.verticalClimbArms.setMotionMagic(RobotContainer.verticalClimbArms.getVerticalClimbLeftTalonPosition(), 8000, 8000, 0.1);
      RobotContainer.verticalClimbArms.setMotionMagic(RobotContainer.verticalClimbArms.getVerticalClimbLeftTalonPosition(), 8000, 8000, 0);
      return true;
    } 
    
    else{
      if (isButtonPressed){
        double sensorLeftDistance = RobotContainer.verticalClimbArms.getVerticalClimbLeftTalonPosition();
        double sensorRightDistance = RobotContainer.verticalClimbArms.getVerticalClimbRightTalonPosition();

        double percentLeftError = 100 * (RobotMap.verticalClimbDistance - sensorLeftDistance)/RobotMap.verticalClimbDistance;
        double percentRightError = 100 * (RobotMap.verticalClimbDistance - sensorRightDistance)/RobotMap.verticalClimbDistance;
        
        SmartDashboard.putNumber("percentErrorVerticalClimbLeft", percentLeftError);
      //even though it is desired to achieve error < 1%, it depends on PID tuning, sometimes it is always achieable
          if ((percentLeftError < 0.5 || percentLeftError < 0 ) && (percentRightError < 0.5 || percentRightError < 0 ))
          //if (percentLeftError < 0.9 || percentLeftError < 0 )
          return true;
        }
        }
    return false;
  }
}
