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



public class ManualTurretTurningCommand extends CommandBase {
  /** Creates a new TurretTurningCommand. */
  private PhotonCamera camera = new PhotonCamera("Limelight Local");
  private boolean hasTargetEver = false;
  private double lastYawAngleByPhotonvision = 0;
  private double driveStraightAngleByPhotonvision = 0;




  public ManualTurretTurningCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSpark);
    //addRequirements(RobotContainer.verticalMotionArm);
    camera.setLED(VisionLEDMode.kOff);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.turretSpark.resetEncoders();
    //RobotContainer.verticalMotionArm.setMotionMagic(RobotMap.armTurretUpDistance, 8000, 8000, 0.1);

    //double[] yawPitchRollArray = new double[3];
    //RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);
    //RobotContainer.turretSpark.getSensorReading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //RobotContainer.oi.turretButton.

    double yawFromPhotonvision=0;
    double[] yawPitchRollArray;
    yawPitchRollArray = new double[3];

    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);

    if(RobotContainer.oi.turretRightPOV.getAsBoolean() == true){
      RobotContainer.turretSpark.manualTurretControl(-0.25 * RobotContainer.turretSpark.maxRPM);
    }

    else if(RobotContainer.oi.turretLeftPOV.getAsBoolean() == true){
      RobotContainer.turretSpark.manualTurretControl(0.25 * RobotContainer.turretSpark.maxRPM);
    }

    else if (RobotContainer.oi.turretHomeButton.getAsBoolean() == true){
      RobotContainer.turretSpark.goHome();
    }
    
    
/*
     else if(RobotContainer.oi.limelightButton.getAsBoolean() )  {
        
        // drive towards the april tag 
        if ( hasTargets == true) {
          joystickRightX = (-1) * yawFromAprilTag * RobotMap.driveToAprilTagProportion;
          driveStraightFlag = false;
          System.out.println("AprilTag Button is clicked ... joystickX  ="+joystickRightX +", yawFromAprilTag = "+yawFromAprilTag);
        }
        else {
          if (hasTargetEver == true) {
            double vinniesError = driveStraightAngleByAprilTag - yawPitchRollArray[0];
            joystickRightX = vinniesError * RobotMap.driveStraightProportion;
            driveStraightFlag = true;
          }
          else {
            driveStraightFlag = false;
          }
        }
      }
      else if(!RobotContainer.oi.limelightButton.getAsBoolean()){
        driveStraightFlag = false;
      }
    
      */
      else if(RobotContainer.oi.photonvisionButton.getAsBoolean() )  {
        //PhotonVision stuff
        var result = camera.getLatestResult();
            
        boolean hasTargets = result.hasTargets();
        if(hasTargets == false) {
          //System.out.println("no target " );
        }
        else {

          // System.out.println("has target " );
          // System.out.println("hasTargest =  " + hasTargets);
           //List<PhotonTrackedTarget> targets = result.getTargets();
           hasTargetEver = true;
           PhotonTrackedTarget target = result.getBestTarget();
           double yaw = target.getYaw();
           lastYawAngleByPhotonvision = yaw;
           double pitch = target.getPitch();
           double area = target.getArea();
           double skew = target.getSkew();
           Transform3d pose = target.getBestCameraToTarget();
           List<TargetCorner> corners = target.getDetectedCorners();
          
           int targetID = target.getFiducialId();
           double poseAmbiguity = target.getPoseAmbiguity();
          // System.out.println("yaw = " + yaw + ", area"+ area + ", pitch = " + pitch + ", targetID =" + targetID);
           yawFromPhotonvision = yaw;
           // figure out the target position based on the angle. not sure if it's positive or negative
           driveStraightAngleByPhotonvision = RobotContainer.turretSpark.getSensorReading() - (yaw / 360);
        }
        
        if ( hasTargets == true) {
          double movePower;
          movePower = (-1) * yawFromPhotonvision * RobotMap.driveToPhotonvisionProportion;
          if(movePower > 0.2){
            movePower = 0.2;
          }
          if(movePower < -0.2){
            movePower = -0.2;
          }
          System.out.println("Photonvision Button is clicked ... movePower  ="+ movePower +", yawFromAprilTag = "+yawFromPhotonvision);
          RobotContainer.turretSpark.manualTurretControl(movePower);
        }
        else {
          double movePower;
          if (hasTargetEver == true) {
            double vinniesError = driveStraightAngleByPhotonvision - RobotContainer.turretSpark.getSensorReading();
            movePower = vinniesError * RobotMap.driveToPhotonvisionByPositionProportion;
            if(movePower > 0.2){
              movePower = 0.2;
            }
            if(movePower < -0.2){
              movePower = -0.2;
            }
          }
         
        }


      }

      else {
        RobotContainer.turretSpark.manualTurretControl(0);
        RobotContainer.turretSpark.holdRotations = RobotContainer.turretSpark.getSensorReading();
      }
  }
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turretSpark.manualTurretControl(0);
    //RobotContainer.verticalMotionArm.setMotionMagic(0, 8000, 8000);
    RobotContainer.turretSpark.holdRotations = RobotContainer.turretSpark.getSensorReading();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (RobotContainer.turret.turretTickCount() >= RobotMap.turretTickAmount){
      //RobotContainer.turret.zeroSensor();
      //return true;
    //}
    return false;
  }
}
