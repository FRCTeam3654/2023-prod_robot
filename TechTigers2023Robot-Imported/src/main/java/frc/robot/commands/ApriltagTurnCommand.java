// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot;
import frc.robot.RobotMap;

import org.photonvision.PhotonCamera;

//  import java.math.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class ApriltagTurnCommand extends CommandBase {
  private double startTime90degree = 0;
  private boolean turn90Flag = false;//indicated the first time in loop. used to set initial conditions
  private double turnEndpointAngle = 0;//vinnies desired finish endpoint angle 
  private double vinniesError = 2;//greater than 1 so it doesn't get triggered until the error is correct
  private double desiredTurnAngle = 0; 
 // private PhotonCamera camera = new PhotonCamera("photonvision");

  /** Creates a new ApriltagTurnCommand. */
  public ApriltagTurnCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {
      double turn90X;
      double turn90Y;
      double [] yawPitchRollArray;
      yawPitchRollArray = new double [3];

      RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);
      
      //var result = camera.getLatestResult();
      //boolean hasTargets = result.hasTargets();
      //System.out.println("hasTargest =  " + hasTargets);







        if (!turn90Flag){
          if (RobotContainer.oi.turnLeft180Button.getAsBoolean()){
            desiredTurnAngle = 180;
          }
          if (RobotContainer.oi.turnRight180Button.getAsBoolean()){
            desiredTurnAngle = -180;
          }
          if (RobotContainer.oi.turnLeft90Button.getAsBoolean()){
            desiredTurnAngle = 90;
          }
          if (RobotContainer.oi.turnRight90Button.getAsBoolean()){
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
          turn90X = vinniesError * 0.007;
          }
        }
        //turn90X = Math.min(1, turn90X);
        //turn90X = Math.max(-1, turn90X);
        turn90X = Math.min(0.5, turn90X);
        turn90X = Math.max(-0.5, turn90X);
        turn90Y = 0;
        //RobotContainer.drive.setArcade(turn90X + 0.07, turn90Y, 0.3, false);
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
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
}
