/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID;
//import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class BothJoystickDriveCommand extends CommandBase {
  private boolean driveStraightFlag = false;
  private double driveStraightAngle = 0;
  private double initialPitch = 0;
  private boolean isBackDriveStarted = false;
  private double driveStraightAngleByAprilTag = 0;
  private double backDriveStartTime = 0;
  private boolean hasTargetEver = false;
  public static double initialYawAngle = 0; //set once at beginning of auto
  private double lastYawAngleByAprilTag = 0;
  public double getLeftTriggerAxis;

  private int backDriveCount = 0;// continous backout count. if > 3 , stop backout
  //private PhotonCamera camera = new PhotonCamera("photonvision");
  private PhotonCamera camera = new PhotonCamera("Limelight Local");

  public BothJoystickDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    addRequirements(RobotContainer.drive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    double[] yawPitchRollArray = new double[3];
    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);
    initialPitch = yawPitchRollArray[1];
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double joystickLeftX;
    double joystickLeftY;
    double joystickRightX;
    double joystickRightY;
    double yawFromAprilTag=0;
    double[] yawPitchRollArray;
    yawPitchRollArray = new double[3];
    joystickLeftX = (RobotContainer.oi.driverStick.getLeftX() * -1);
    joystickLeftY = (RobotContainer.oi.driverStick.getLeftY() * -1);
    joystickRightX = (RobotContainer.oi.driverStick.getRightX() * -1);
    joystickRightY = (RobotContainer.oi.driverStick.getRightY() * -1);

    joystickLeftX = handleDeadband(joystickLeftX, RobotMap.joystickDeadBand);
    joystickLeftY = handleDeadband(joystickLeftY, RobotMap.joystickDeadBand);
    joystickRightX = handleDeadband(joystickRightX, RobotMap.joystickDeadBand);
    joystickRightY = handleDeadband(joystickRightY, RobotMap.joystickDeadBand);

    double nonTurboSpeed = RobotMap.nonTurboMultiplierForward;


    // This is to activate turbo mode. If the button is pressed, turbo mode is on
    if(RobotContainer.oi.driverStick.getLeftTriggerAxis() > 0.4){
      //System.out.println("TURBO");
    }

    else {
      joystickLeftX = joystickLeftX * RobotMap.nonTurboMultiplierTurn;
      joystickRightY = joystickRightY * nonTurboSpeed;
      joystickRightX = joystickRightX * RobotMap.nonTurboMultiplierTurn;
      joystickLeftY = joystickLeftY * nonTurboSpeed;
   }

    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);
    
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
          lastYawAngleByAprilTag = yaw;
          double pitch = target.getPitch();
          double area = target.getArea();
          double skew = target.getSkew();
          Transform3d pose = target.getBestCameraToTarget();
          List<TargetCorner> corners = target.getDetectedCorners();
         
          int targetID = target.getFiducialId();
          double poseAmbiguity = target.getPoseAmbiguity();
         // System.out.println("yaw = " + yaw + ", area"+ area + ", pitch = " + pitch + ", targetID =" + targetID);
          yawFromAprilTag = yaw;
          // figure out the drive straight angles
          driveStraightAngleByAprilTag = yawPitchRollArray[0] - yaw;
        }

    SmartDashboard.putNumber("Pitch", yawPitchRollArray[1]);
    //System.out.println("Joystick X ="+ joystickX);
    //System.out.println("Joystick Y ="+ joystickY);

    //Drive Straight Function
    if (RobotContainer.oi.driverStick.getRightTriggerAxis() > 0.4) { //drive straight button
      // joystickX = 0;
      //if (!driveStraightFlag) {
       // driveStraightAngle = yawPitchRollArray[0];
       // driveStraightFlag = true;
     // }
      //double vinniesError = driveStraightAngle - yawPitchRollArray[0];
      //joystickRightX = vinniesError * RobotMap.driveStraightProportion;
     nonTurboSpeed = 0.4;
    }
    //X is the left-right axis
    //Limelight/Apriltag Button
    else {
      if  (RobotContainer.oi.limelightButton.getAsBoolean() )  {
        
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
      else {
        driveStraightFlag = false;
      }
      nonTurboSpeed = RobotMap.nonTurboMultiplierForward;
    }

      //System.out.println("X=" + joystickLeftX + "Y=" + joystickLeftY);
      RobotContainer.drive.setArcade(joystickRightX, joystickLeftY);

      // Dashboard features for Joystick x and y values and right and left encoders
      SmartDashboard.putNumber("Joystick Left X: ", joystickLeftX);
      SmartDashboard.putNumber("Joystick Left Y: ", joystickLeftY);
      SmartDashboard.putNumber("Joystick Right X: ", joystickRightX);
      SmartDashboard.putNumber("Joystick Right Y: ", joystickRightY);
      SmartDashboard.putNumber("Left Encoder", RobotContainer.drive.leftFrontTalon.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Right Encoder", RobotContainer.drive.rightFrontTalon.getSelectedSensorVelocity());
      //SmartDashboard.putNumber("Yaw: ", yawPitchRollArray[0]);
    }
 // }

  // Deadband makes the center of the joystick have leeway on absolute 0
  public double handleDeadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}