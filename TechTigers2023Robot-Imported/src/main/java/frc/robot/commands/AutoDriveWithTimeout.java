// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class AutoDriveWithTimeout extends CommandBase {
  /** Creates a new AutoDriveWithTimeout. */
  private double autoDriveTimer = 0;
  private double drivePercentPower = 0.4;
  private double driveTimeout = 2;
  private double maxDistance = 0;
  private double driveStraightYaw = 0;
  private static double pulsesPerInch = 0.0254 / Constants.DriveConstants.kEncoderDistancePerPulse;

  public AutoDriveWithTimeout() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
  }

  public AutoDriveWithTimeout(double percentPower, double autodrive_timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivePercentPower = Math.abs(percentPower);
    driveTimeout = autodrive_timeout;
    addRequirements(RobotContainer.drive);
  }

  public AutoDriveWithTimeout(double percentPower, double autodrive_timeout, double maxDistanceInInch ) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivePercentPower = Math.abs(percentPower);
    driveTimeout = autodrive_timeout;
    this.maxDistance =  maxDistanceInInch * pulsesPerInch;
    addRequirements(RobotContainer.drive);
  }

  public AutoDriveWithTimeout(double percentPower, double autodrive_timeout, double maxDistanceInInch,  double driveStraightYaw) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivePercentPower = Math.abs(percentPower);
    driveTimeout = autodrive_timeout;
    this.maxDistance  = maxDistanceInInch * pulsesPerInch;
    this.driveStraightYaw = driveStraightYaw;
    addRequirements(RobotContainer.drive);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoDriveTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] yawPitchRollArray = new double[3];
    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);
    double vinniesError = driveStraightYaw - yawPitchRollArray[0]; // aimed at orignal 0 degree 
    double joystickX = vinniesError * RobotMap.driveStraightProportion;
    double joystickY = drivePercentPower;
    // need add logic to slow down at the end
    if( maxDistance < 0 ) {
       joystickY = (-1) * drivePercentPower;
        if((RobotContainer.drive.getAverageEncoderDistance() - maxDistance) < (10 * pulsesPerInch ) && (RobotContainer.drive.getAverageEncoderDistance() - maxDistance) > 0 ) {
          joystickY = -0.2; // use much less power in last 10 inches
        }
        else if( (RobotContainer.drive.getAverageEncoderDistance() - maxDistance) <= 0 ) {
          joystickY = 0;
        }
    } 
    else if( maxDistance > 0 ) {
      if((RobotContainer.drive.getAverageEncoderDistance() - maxDistance) > (-10 * pulsesPerInch ) && (RobotContainer.drive.getAverageEncoderDistance() - maxDistance) < 0 ) {
        joystickY = 0.2; // use much less power in last 10 inches
      }
      else if( (RobotContainer.drive.getAverageEncoderDistance() - maxDistance) >= 0 ) {
        joystickY = 0;
      }
  }

    RobotContainer.drive.setArcade(joystickX, joystickY, 0.5, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.setArcade(0, 0, drivePercentPower, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(autoDriveTimer + driveTimeout < Timer.getFPGATimestamp()) {   
      return true;
    }
    else if(maxDistance < 0) {
      if( (RobotContainer.drive.getAverageEncoderDistance() - maxDistance) <= 0 ) {
         return true;
      }
    }
    else if (maxDistance >= 0) {
      if( (RobotContainer.drive.getAverageEncoderDistance() - maxDistance) >= 0 ) {
          return true;
      }
    }

    return false;
  }
}
