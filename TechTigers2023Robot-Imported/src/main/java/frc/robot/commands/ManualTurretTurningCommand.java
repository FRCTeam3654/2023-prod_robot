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
  private double turretTimer;
  private boolean isLeftPressed = false;
  private boolean isRightPressed = false;
  private boolean isHomeButtonPressed = false;
  private boolean timeStarted = false;
  public static int mode = 0; // 1 : turretRightPOV, 2:  turretLeftPOV ,  3: turretHomeButton
  private double turretTurnTimeout = 0.9;
  private double maxVoltage = 1.5; // drive the turrent to home


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
      if( timeStarted == false) {
        turretTimer = Timer.getFPGATimestamp();
        timeStarted = true;
        mode = 1;
      }
      RobotContainer.turretSpark.manualTurretControl(-1);
    }

    //if(isRightPressed == true){
    //}

    else if(RobotContainer.oi.turretLeftPOV.getAsBoolean() == true){
      if( timeStarted == false) {
        turretTimer = Timer.getFPGATimestamp();
        timeStarted = true;
        mode = 2;
      }
      RobotContainer.turretSpark.manualTurretControl(1); //maxRPM
      
    }

    //if(isLeftPressed == true){
    //}

    //else if (RobotContainer.oi.turretLeftPOV.getAsBoolean() == false){
     // isLeftPressed = false;
  //  }
    
    //else if (RobotContainer.oi.turretRightPOV.getAsBoolean() == false){
      //isRightPressed = false;
    //}

    else if (RobotContainer.oi.turretHomeButton.getAsBoolean() == true || isHomeButtonPressed == true){
      mode = 3;
      //RobotContainer.turretSpark.goHome();
      //turretTimer = Timer.getFPGATimestamp();

       
      if( timeStarted == false) {
        turretTimer = Timer.getFPGATimestamp();
        timeStarted = true; 
        isHomeButtonPressed = true;
      }

      // positive voltage move towards left,  2.88 is at left 90 degree
      double setPoint = maxVoltage; // in voltages
      

      double currrentReading = RobotContainer.turretSpark.getSensorReading();

      // implement P (=-1) part of PID: proportional x error 
      setPoint =  (-0.8) * currrentReading;
      if( setPoint > maxVoltage) {
        setPoint = maxVoltage;
      }
      else if ( setPoint < ((-1) * maxVoltage)) {
        setPoint = (-1) * maxVoltage;
      }
      else if (Math.abs(currrentReading) < 0.2) {
        setPoint = 0;
      }
      
      RobotContainer.turretSpark.manualTurretControl(setPoint);


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
      /* 
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
      */
      else {
        RobotContainer.turretSpark.manualTurretControl(0);
        //RobotContainer.turretSpark.holdRotations = RobotContainer.turretSpark.getSensorReading();
      }
  }
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    RobotContainer.turretSpark.manualTurretControl(0);

    //RobotContainer.verticalMotionArm.setMotionMagic(0, 8000, 8000);
    if( mode == 3){
      RobotContainer.turretSpark.holdRotations = 0; // gohome is always 0, even if the reading is not zero after timeout
    }
    else {
      RobotContainer.turretSpark.holdRotations = RobotContainer.turretSpark.getSensorReading();
    }
    mode = 0;
    isLeftPressed = false;
    isRightPressed = false;
    isHomeButtonPressed = false;
    timeStarted = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (RobotContainer.turret.turretTickCount() >= RobotMap.turretTickAmount){
      //RobotContainer.turret.zeroSensor();
      //return true;
    //}
   // if(RobotContainer.oi.turretLeftPOV.getAsBoolean() == false && RobotContainer.oi.turretRightPOV.getAsBoolean() == false){
    //  isLeftPressed = false;
     // isRightPressed = false;
      //return true;
   // }

    double timeoutvalue = turretTurnTimeout;
    if (mode == 3) {
      timeoutvalue = 1.5; // not sure if gohome need a little more time for returning to home (0)
      if( Math.abs(RobotContainer.turretSpark.getSensorReading()) < 0.1 ) {
          // noted: reading 2.8 at robot is about 90 degree, so 0.1 is about 3 degree 
          //RobotContainer.turretSpark.holdRotations = RobotContainer.turretSpark.getSensorReading();
          return true;
      } 
    }
    else {
      double currrentReading = RobotContainer.turretSpark.getSensorReading();
      if( Math.abs(currrentReading) > 2.8 ) {
        // force it to not move beyond +/- 90 degree
        if( currrentReading  > 2.8  && mode == 2)  {
          return true;
        }
        else if(currrentReading  < -2.8  && mode == 1 ) {
          return true;
        }
          
      }

    }

    if((turretTimer + timeoutvalue) < Timer.getFPGATimestamp()){
      return true;
    }
    return false;
  }
}
