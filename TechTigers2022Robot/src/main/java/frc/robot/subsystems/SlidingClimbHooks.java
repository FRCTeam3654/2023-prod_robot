// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
//import java.io.*;
//import java.util.ArrayList;
import com.ctre.phoenix.motion.*;
//import java.util.concurrent.atomic.AtomicInteger;
//import frc.robot.Constants;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.math.controller.PIDController;

public class SlidingClimbHooks extends SubsystemBase {
  /** Creates a new SlidingClimbHooks. */
  private WPI_TalonFX climbHookTalonLeft = new WPI_TalonFX(RobotMap.climbHookTalonLeftID);
  private WPI_TalonFX climbHookTalonRight = new WPI_TalonFX(RobotMap.climbHookTalonRightID);
  public SlidingClimbHooks() {
    climbHookTalonLeft.configFactoryDefault();
    climbHookTalonRight.configFactoryDefault();

    climbHookTalonLeft.set(ControlMode.PercentOutput, 0);
    climbHookTalonRight.set(ControlMode.PercentOutput, 0);

    climbHookTalonLeft.configNeutralDeadband(0.0,30);
    climbHookTalonRight.configNeutralDeadband(0.0,30);

    climbHookTalonLeft.configClosedloopRamp(1);
    climbHookTalonRight.configOpenloopRamp(1);
    climbHookTalonRight.configClosedloopRamp(1);
    climbHookTalonLeft.configOpenloopRamp(1);

    climbHookTalonRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);
    climbHookTalonLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);

    climbHookTalonLeft.selectProfileSlot(0,0);
    climbHookTalonRight.selectProfileSlot(0,0);

    climbHookTalonLeft.config_kF(0,0.045,30);// 0.045
    climbHookTalonLeft.config_kP(0,0.049,30); //0.095 //0.049
    climbHookTalonLeft.config_kI(0,0,30);
    climbHookTalonLeft.config_kD(0,0,30);

    climbHookTalonRight.config_kF(0,0.045,30); // 0.045
    climbHookTalonRight.config_kP(0,0.049,30); //0.095 //0.049
    climbHookTalonRight.config_kI(0,0,30);
    climbHookTalonRight.config_kD(0,0,30);
    
    climbHookTalonLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    climbHookTalonLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

    climbHookTalonRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    climbHookTalonRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

    climbHookTalonLeft.setInverted(false);
    climbHookTalonRight.setInverted(true);

    climbHookTalonLeft.setNeutralMode(NeutralMode.Brake);
    climbHookTalonRight.setNeutralMode(NeutralMode.Brake);
    
    climbHookTalonLeft.configNominalOutputForward(0, 30);
    climbHookTalonLeft.configNominalOutputReverse(0, 30);
    climbHookTalonLeft.configPeakOutputForward(1, 30);
    climbHookTalonLeft.configPeakOutputReverse(-1, 30);

    climbHookTalonRight.configNominalOutputForward(0, 30);
    climbHookTalonRight.configNominalOutputReverse(0, 30);
    climbHookTalonRight.configPeakOutputForward(1, 30);
    climbHookTalonRight.configPeakOutputReverse(-1, 30);
  
    climbHookTalonLeft.configMotionCruiseVelocity(8000, 30);
    climbHookTalonLeft.configMotionAcceleration(8000, 30);

    climbHookTalonRight.configMotionCruiseVelocity(8000, 30);
    climbHookTalonRight.configMotionAcceleration(8000, 30);

    climbHookTalonLeft.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);
    climbHookTalonRight.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);

    zeroSensors();
}
  /** Zero Quadrature Encoders on Talons */
	public void zeroSensors() {
    climbHookTalonLeft.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
    climbHookTalonRight.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
    //pigeonVinnie.setYaw(0, RobotMap.pidLoopTimeout);
    //pigeonVinnie.setFusedHeading(0, RobotMap.pidLoopTimeout);
    //pigeonVinnie.setAccumZAngle(0, RobotMap.pidLoopTimeout);
  
    System.out.println("[Quadrature Encoders] All drive sensors are zeroed.\n");
}
public void resetMotors(){
  zeroSensors();
  climbHookTalonLeft.set(ControlMode.PercentOutput, 0);
  climbHookTalonRight.set(ControlMode.PercentOutput, 0);
}
public double getClimbHookTalonLeftPosition(){
  return climbHookTalonLeft.getSelectedSensorPosition(0);
}
public double getClimbHookTalonRightPosition(){
  return climbHookTalonRight.getSelectedSensorPosition(0);
} 
public void driveClimbMotors(double percentOutput){
  climbHookTalonLeft.set(ControlMode.PercentOutput, percentOutput);
  climbHookTalonRight.set(ControlMode.PercentOutput, percentOutput);
}

public void setMotionMagic(double distance, int cruiseVelocity, int accelerationVelocity) {
  setMotionMagic (distance, 0.0, cruiseVelocity,  accelerationVelocity, false);
}

public void setMotionMagic(double distance, double turn_angle, int cruiseVelocity, int accelerationVelocity, boolean useAuxPID) {
  climbHookTalonLeft.configMotionCruiseVelocity(cruiseVelocity, RobotMap.pidLoopTimeout);
  climbHookTalonLeft.configMotionAcceleration(accelerationVelocity, RobotMap.pidLoopTimeout);

  climbHookTalonRight.configMotionCruiseVelocity(cruiseVelocity, RobotMap.pidLoopTimeout);
  climbHookTalonRight.configMotionAcceleration(accelerationVelocity, RobotMap.pidLoopTimeout);

  climbHookTalonLeft.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);
  climbHookTalonRight.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);

  if( useAuxPID == false ) {
    climbHookTalonLeft.set(ControlMode.MotionMagic, distance);
    climbHookTalonRight.set(ControlMode.MotionMagic, distance);
  }
  else {
    // the following are new for Arc setup
    climbHookTalonLeft.selectProfileSlot(RobotMap.kTurnAutonomousSlotIDx, RobotMap.PID_TURN);
    climbHookTalonRight.selectProfileSlot(RobotMap.kTurnAutonomousSlotIDx, RobotMap.PID_TURN);

    climbHookTalonRight.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, turn_angle);		
    climbHookTalonLeft.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, turn_angle);
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
