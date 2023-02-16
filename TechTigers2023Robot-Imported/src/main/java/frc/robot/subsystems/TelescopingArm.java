// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.DemandType;

public class TelescopingArm extends SubsystemBase {
  /** Creates a new TelescopingArm. */
  private TalonSRX verticalClimbLeftTalon = new TalonSRX(RobotMap.armTalonID);

  public double leftSpeed; 

    

  public TelescopingArm() {
    verticalClimbLeftTalon.configFactoryDefault();

    verticalClimbLeftTalon.setNeutralMode(NeutralMode.Brake);

    verticalClimbLeftTalon.setInverted(true);
    
    //verticalClimbRightTalon.follow(verticalClimbLeftTalon);

    verticalClimbLeftTalon.configNeutralDeadband(0.00, RobotMap.kTimeoutMs);
    

   

    verticalClimbLeftTalon.config_kF(0,0.045,30);// 0.045
    verticalClimbLeftTalon.config_kP(0,0.049,30); //0.095 //0.049
    verticalClimbLeftTalon.config_kI(0,0,30);
    verticalClimbLeftTalon.config_kD(0,0,30);

   

    verticalClimbLeftTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.kTimeoutMs);
    verticalClimbLeftTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.kTimeoutMs);
   
    /* set the peak and nominal outputs */
    verticalClimbLeftTalon.configNominalOutputForward(0, RobotMap.kTimeoutMs);
    verticalClimbLeftTalon.configNominalOutputReverse(0, RobotMap.kTimeoutMs);
    verticalClimbLeftTalon.configPeakOutputForward(1, RobotMap.kTimeoutMs);
    verticalClimbLeftTalon.configPeakOutputReverse(-1, RobotMap.kTimeoutMs);

   

    verticalClimbLeftTalon.configMotionCruiseVelocity(8000, 30);
    verticalClimbLeftTalon.configMotionAcceleration(8000, 30);

   

    verticalClimbLeftTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);
    
    zeroSensors();
  }
  public void resetMotors(){
    zeroSensors();
    verticalClimbLeftTalon.set(ControlMode.PercentOutput, 0);
  }
  public void zeroSensors() {
    verticalClimbLeftTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    System.out.println("[Quadrature Encoders] All drive sensors are zeroed.\n");
  }
  public void setArcade(double velocity, double turn){
    mercyArcadeDrive(velocity, turn);
  }
  public void mercyArcadeDrive(double joystickX, double joystickY){
    double radiusPower = Math.hypot(joystickX, joystickY);
    double initAngle = Math.atan2(joystickX, joystickY);
    initAngle = initAngle + Math.PI/4;
    leftSpeed = radiusPower*Math.cos(initAngle);
    leftSpeed = leftSpeed*1.414;
    
    if (leftSpeed > 1) {
      leftSpeed = 1;
    }

    if (leftSpeed < -1) {
      leftSpeed = -1;
    }

      if (RobotMap.climbClosedLoopMode ) {
        //closed loop
        double targetVelocity_UnitsPer100ms_left = leftSpeed * RobotMap.maximumVelocityFalcon;
        verticalClimbLeftTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms_left);
      }
      
    else {
      //open loop
      verticalClimbLeftTalon.set(ControlMode.PercentOutput, leftSpeed);
   }
  }
  public void setMotionMagic(double distance, int cruiseVelocity, int accelerationVelocity ) {
    setMotionMagic( distance, cruiseVelocity,  accelerationVelocity, 0) ;
   }

  public void setMotionMagic(double distance, int cruiseVelocity, int accelerationVelocity, double arbitraryFeedForwardValue) {
        verticalClimbLeftTalon.configMotionCruiseVelocity(cruiseVelocity, RobotMap.pidLoopTimeout);
        verticalClimbLeftTalon.configMotionAcceleration(accelerationVelocity, RobotMap.pidLoopTimeout);
      
        verticalClimbLeftTalon.configMotionAcceleration(accelerationVelocity, RobotMap.pidLoopTimeout);
      
        verticalClimbLeftTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);

        if( Math.abs(arbitraryFeedForwardValue) > 0.001) {
          // add  DemandType.ArbitraryFeedForward to hold the vertical arm in position
          verticalClimbLeftTalon.set(ControlMode.MotionMagic, distance,  DemandType.ArbitraryFeedForward, arbitraryFeedForwardValue);
        }
        else {
          verticalClimbLeftTalon.set(ControlMode.MotionMagic, distance);
        }
}
public void karenaArcadeDrive(double joystickX, double joystickY){
  //forward becomes postive
  //joystickY *= -1;

    //System.out.println("ElevatorX = " + joystickX + "ElevatorY = " + joystickY);

    //The java.lang.Math.asin() returns the arc sine of an angle in between -pi/2 and pi/2.
    //double radiusPower = Math.sqrt(joystickX*joystickX + joystickY*joystickY);
    double radiusPower = Math.hypot(joystickX, joystickY);
    if (radiusPower > 0.2){
      //sin value range is from -0.5 pi to 0.5 pi
      //double initAngle = Math.atan2(joystickX, joystickY);
      double initAngle = Math.asin(Math.abs(joystickY)/radiusPower);
      double initAngleDegree = initAngle * RobotMap.radianConversionToDegree;
      if(joystickX > 0 && joystickY > 0  ) {
        // do nothing
      }
      else if(joystickX < 0 && joystickY > 0  ) {
        initAngleDegree = 180 - initAngleDegree;
      } 
      else if(joystickX < 0 && joystickY < 0  ) {
        initAngleDegree = 180 + initAngleDegree;
      } 
      else if(joystickX > 0 && joystickY < 0  ) {
        initAngleDegree = 360 - initAngleDegree;
      } 

/*
      else if (isAboutEqual(joystickY, 0.0) && joystickX > 0 ) {
        initAngleDegree = 0;
      }
      else if (isAboutEqual(joystickX, 0.0) && joystickY > 0 ) {            
        initAngleDegree = 90;
      }
      else if (isAboutEqual(joystickY, 0.0) && joystickX < 0 ) {            
        initAngleDegree = 180;
      }
      else if (isAboutEqual(joystickX, 0.0) && joystickY < 0 ) {            
        initAngleDegree = 270;
      }
*/

      System.out.println("Angle = " + initAngleDegree);
      SmartDashboard.putNumber("VerticalClimbAngleDegree", initAngleDegree);

if(  isAboutAngle (initAngleDegree,0) ) {
  //  joystick is shifted to the right within angle deadband, move right arm up
  //verticalClimbLeftTalon.set(ControlMode.PercentOutput, 0);
  //verticalClimbRightTalon.set(ControlMode.PercentOutput,(RobotMap.climbSpeed));
 }
 else if(  isAboutAngle (initAngleDegree,90) ) {
  //  joystick upwards within deadband, move both arms up
  verticalClimbLeftTalon.set(ControlMode.PercentOutput, RobotMap.climbSpeed);
 }
 else if(  isAboutAngle (initAngleDegree,180) ) {
  //  joystick is shifted to the left within angle deadband, move left arm up
  //verticalClimbLeftTalon.set(ControlMode.PercentOutput, RobotMap.climbSpeed);
  //verticalClimbRightTalon.set(ControlMode.PercentOutput, 0);
 }
 else if(  isAboutAngle (initAngleDegree,270) ) {
  //  joystick downwards within angle deadband, move both arms down
  verticalClimbLeftTalon.set(ControlMode.PercentOutput, (-1 * RobotMap.climbSpeed));
 }


      /*
      if (initAngleDegree > (-1 * RobotMap.angleDeadBand) && initAngleDegree < RobotMap.angleDeadBand){
        //joystick is shifted to the right within angle deadband, does nothing
        verticalClimbLeftTalon.set(ControlMode.PercentOutput, 0);
        verticalClimbRightTalon.set(ControlMode.PercentOutput, 0);
      }
      else if (initAngleDegree > RobotMap.angleDeadBand && initAngleDegree < (90 - RobotMap.angleDeadBand)){
        //joystick is shifted to ther right above angle deadband, only the right arm moves up
        verticalClimbLeftTalon.set(ControlMode.PercentOutput, 0);
        verticalClimbRightTalon.set(ControlMode.PercentOutput,(-1 * RobotMap.climbSpeed));
      }
      else if (initAngleDegree > (90 - RobotMap.angleDeadBand) && initAngleDegree < (90 + RobotMap.angleDeadBand)){
        //shift upwards within deadband, both arms move up
        verticalClimbLeftTalon.set(ControlMode.PercentOutput, RobotMap.climbSpeed);
        verticalClimbRightTalon.set(ControlMode.PercentOutput, (-1 * RobotMap.climbSpeed));
      }
      else if (initAngleDegree > (90 + RobotMap.angleDeadBand) && initAngleDegree < (180 - RobotMap.angleDeadBand)){
        //shifted to the left within deadband, where only the left arm moves up
        verticalClimbLeftTalon.set(ControlMode.PercentOutput, RobotMap.climbSpeed);
        verticalClimbRightTalon.set(ControlMode.PercentOutput, 0);
      }
      else if (initAngleDegree > (180 - RobotMap.angleDeadBand) && initAngleDegree < (180 + RobotMap.angleDeadBand)){
        //shift to the left, within deadband so it does nothing
        verticalClimbLeftTalon.set(ControlMode.PercentOutput, 0);
        verticalClimbRightTalon.set(ControlMode.PercentOutput, 0);
      }
      else if (initAngleDegree > (180 + RobotMap.angleDeadBand) && initAngleDegree < (270 - RobotMap.angleDeadBand)){
        //shift downwards to the left outside the deadband, so only the left arm moves down
        verticalClimbLeftTalon.set(ControlMode.PercentOutput,  (-1 * RobotMap.climbSpeed));
        verticalClimbRightTalon.set(ControlMode.PercentOutput, 0);
      }
      else if (initAngleDegree > (270 - RobotMap.angleDeadBand) && initAngleDegree < (270 + RobotMap.angleDeadBand)){
        //shift downwards within deadband, so both arms move downwards
        verticalClimbLeftTalon.set(ControlMode.PercentOutput, (-1 * RobotMap.climbSpeed));
        verticalClimbRightTalon.set(ControlMode.PercentOutput, RobotMap.climbSpeed);
      }
      else if (initAngleDegree > (270 + RobotMap.angleDeadBand) && initAngleDegree < (360 - RobotMap.angleDeadBand)){
        //shift downwards within deadband, so only the right arm moves down
        verticalClimbLeftTalon.set(ControlMode.PercentOutput, 0);
        verticalClimbRightTalon.set(ControlMode.PercentOutput, RobotMap.climbSpeed);
      }
      */
    }

    else {
      verticalClimbLeftTalon.set(ControlMode.PercentOutput, 0);
    }
  }

    boolean isAboutEqual(double input, double to) {
      boolean ret = false;
      double diff =  Math.abs(input) - Math.abs(to);
      if (Math.abs(diff) < 0.001) {
         ret = true;
      }
      return ret;
   }


  boolean isAboutAngle(double inputAngle, double targetAngle) {
  boolean ret = false;
  double diff =  Math.abs(inputAngle) - Math.abs(targetAngle);
  if (Math.abs(diff) < RobotMap.angleDeadBand) {
    ret = true;
  }
  else if ( inputAngle > (360 - RobotMap.angleDeadBand) && inputAngle <= 360) {
    ret = true;
  }
  return ret;
  }

  

public double getVerticalClimbLeftTalonPosition(){
  return verticalClimbLeftTalon.getSelectedSensorPosition(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
