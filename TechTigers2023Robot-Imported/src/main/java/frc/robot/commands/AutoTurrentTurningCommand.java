// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Transform2d;
import org.photonvision.common.hardware.VisionLEDMode;


public class AutoTurrentTurningCommand extends CommandBase {

  private double turretTimer;
  private boolean timeStarted = false;
  public static int mode = 0; // 4: go to certain position,  1 : turretRightPOV, 2:  turretLeftPOV ,  3: turretHomeButton
  private double turretTurnTimeout = 0.9;
  private double targetPosition = 0 ;// positive 2.88 is about 90 degree to the left (facing robot front)
  private double maxVoltage = 1.5; // drive the turret to position


  /** Creates a new AutoTurrentTurningCommand. */
  public AutoTurrentTurningCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSpark);
  }

  public AutoTurrentTurningCommand(int new_mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSpark);
    mode = new_mode;
  }

  public AutoTurrentTurningCommand(int new_mode, double new_timeout, double new_position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSpark);
    mode = new_mode;
    turretTurnTimeout = new_timeout ;
    targetPosition = new_position;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mode = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( mode == 4) {
      if( timeStarted == false) {
        turretTimer = Timer.getFPGATimestamp();
        timeStarted = true;
        //RobotContainer.turretSpark.goToPosition(targetPosition);
        //RobotContainer.turretSpark.goToPositionBySmartMotion((targetPosition);
      }
        // positive voltage move towards left,  2.88 is at left 90 degree
        double setPoint = maxVoltage; // in voltages
        

        double currentReading = RobotContainer.turretSpark.getSensorReading();
        double error = currentReading - targetPosition;

        // implement P (=-1) part of PID: proportional x error 
        setPoint =  (-0.8) * error;
        if( setPoint > maxVoltage) {
          setPoint = maxVoltage;
        }
        else if ( setPoint < ((-1) * maxVoltage)) {
          setPoint = (-1) * maxVoltage;
        }
        else if (Math.abs(error) < 0.2) {
          setPoint = 0;
        }
        
        RobotContainer.turretSpark.manualTurretControl(setPoint);


      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double timeoutvalue = turretTurnTimeout;
    if (mode == 4) {
      if( Math.abs(RobotContainer.turretSpark.getSensorReading() - targetPosition) < 0.1 ) {
          // noted: reading 2.8 at robot is about 90 degree, so 0.1 is about 3 degree 
          return true;
      } 
    }
    
    // force to not turn more than 180 degree
      double currrentReading = RobotContainer.turretSpark.getSensorReading();
    if( Math.abs(currrentReading) > 5.6 ) { 
          return true;  
    }

    

    if((turretTimer + timeoutvalue) < Timer.getFPGATimestamp()){
      return true;
    }

    return false;
  }
}
