/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot;
import frc.robot.RobotMap;
//  import java.math.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class AutoTurn90DegreesCommand extends CommandBase {
  private boolean turn90Flag = false;//indicated the first time in loop. used to set initial conditions
  private double turnEndpointAngle = 0;//vinnies desired finish endpoint angle 
  private double vinniesError = 2;//greater than 1 so it doesn't get triggered until the error is correct
  private double startTime90degree = 0;
  private double desiredTurnAngle = 0; 
  private boolean left90Flag = false;
  private boolean left180Flag = false;
  private boolean right90Flag = false;
  private boolean right180Flag = false;
  private int mode; //0 is left 90, 1 is left 180, 2 is right 90, 3 is right 180

  public AutoTurn90DegreesCommand() {
    // Use requires() here to declare subsystem dependencies
    addRequirements(RobotContainer.drive);
  }

  public AutoTurn90DegreesCommand(int mode) {
    // Use requires() here to declare subsystem dependencies
    addRequirements(RobotContainer.drive);
    this.mode = mode;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  //Only use the left button 180/90
  @Override
  public void execute() {
    double turn90X;
    double turn90Y;
    double [] yawPitchRollArray;
    yawPitchRollArray = new double [3];
    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);
      if (!turn90Flag){
        if (mode == 1){
          desiredTurnAngle = 180;
        }
        else if (mode == 3){
          desiredTurnAngle = -188;
        }
        else if (mode == 0){
          desiredTurnAngle = 70;
        }
        else if (mode == 2){
          desiredTurnAngle = -98;
        }
        turnEndpointAngle = yawPitchRollArray[0] + desiredTurnAngle;
        startTime90degree = Timer.getFPGATimestamp();
        turn90Flag = true;
      }
      vinniesError = turnEndpointAngle - yawPitchRollArray[0];
      if (Math.abs(vinniesError) > 90){   // for large angles we tune it down a bit
        if (vinniesError > 0){
          vinniesError = 60;
          turn90X = vinniesError * RobotMap.turnDegreeProportion*(0.85);
        }
        else {
          vinniesError = -60;
          turn90X = vinniesError * RobotMap.turnDegreeProportion*(0.85);
        }
      }
      else {
        if (Math.abs(desiredTurnAngle) == 180){
          turn90X = vinniesError * 0.004;
        }
        else{
        turn90X = vinniesError * 0.0056;
        }
      }
       //turn90X = Math.min(1, turn90X);
       //turn90X = Math.max(-1, turn90X);
       turn90X = Math.min(0.5, turn90X);
       turn90X = Math.max(-0.5, turn90X);
       turn90Y = 0;
       RobotContainer.drive.setArcade(turn90X + 0.07, turn90Y, 0.3, false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if(Math.abs(vinniesError) < 3){  // Nearly at desired angle
      turn90Flag = false;
      return true;
    }
    if(startTime90degree + RobotMap.turn90DegreeTimeout < Timer.getFPGATimestamp()) {  // to much time to find desired angle
      turn90Flag = false;
      return true;
    }
    if(RobotContainer.oi.turboButton.getAsBoolean()){   // Pressing Turbo buttion stops turning
      turn90Flag = false;
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  left90Flag = false;
  left180Flag = false;
  right90Flag = false;
  right180Flag = false;
  }

  
}
