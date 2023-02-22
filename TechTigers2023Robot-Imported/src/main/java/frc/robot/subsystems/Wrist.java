// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private TalonFX wristTalon = new TalonFX (RobotMap.wristTalonID);
  private double wristTargetPosition;
  
  public Wrist() {
    wristTalon.configFactoryDefault();
    
    
    wristTalon.setNeutralMode(NeutralMode.Brake);
    wristTalon.configNeutralDeadband(0.01, RobotMap.pidLoopTimeout); //during testing was 0.001
    
    wristTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);

    /* Set relevant frame periods to be at least as fast as periodic rate */
		wristTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.pidLoopTimeout);
		wristTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.pidLoopTimeout);

		/* Set the peak and nominal outputs */
		wristTalon.configNominalOutputForward(0, RobotMap.pidLoopTimeout);
		wristTalon.configNominalOutputReverse(0, RobotMap.pidLoopTimeout);
		wristTalon.configPeakOutputForward(1, RobotMap.pidLoopTimeout);
		wristTalon.configPeakOutputReverse(-1, RobotMap.pidLoopTimeout);

		/* Set gains in slot0 - see documentation */
    wristTalon.selectProfileSlot(RobotMap.kShooterSlotIDx, RobotMap.kPIDLoopIDx);
		wristTalon.config_kF(RobotMap.kShooterSlotIDx, RobotMap.wristGainsVelocity.kF, RobotMap.pidLoopTimeout);
		wristTalon.config_kP(RobotMap.kShooterSlotIDx, RobotMap.wristGainsVelocity.kP, RobotMap.pidLoopTimeout);
		wristTalon.config_kI(RobotMap.kShooterSlotIDx, RobotMap.wristGainsVelocity.kI, RobotMap.pidLoopTimeout);
		wristTalon.config_kD(RobotMap.kShooterSlotIDx, RobotMap.wristGainsVelocity.kD, RobotMap.pidLoopTimeout);

		/* Set acceleration and vcruise velocity - see documentation */
		wristTalon.configMotionCruiseVelocity(500, RobotMap.pidLoopTimeout);
		wristTalon.configMotionAcceleration(500, RobotMap.pidLoopTimeout);

		/* Zero the sensor once on robot boot up */    
    zeroSensor();
  }

  public void zeroSensor() {
    wristTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    //System.out.println("wrist Sensor is 0");
  }

  public void wristTurning(double targetPos) {
    if(targetPos > 5000){
      targetPos = 5000;
    }
    if(targetPos < -5000){
      targetPos = -5000;  //adjust 5000 based on experimental limits 
    }
    wristTargetPosition = targetPos;
    wristTalon.set(ControlMode.MotionMagic, targetPos);
    System.out.println(wristTalon.getSelectedSensorVelocity(0) + ",  " + wristTalon.getClosedLoopError(0) + ",  " + wristTalon.getSelectedSensorPosition(0));
  }

  public int wristTickCount(){
    return (int)wristTalon.getSelectedSensorPosition();   
  }

  public void manualwrist(double percentOutput){
    wristTalon.set(ControlMode.PercentOutput, percentOutput);
  }


  public boolean atTargetPosition(){
    if(Math.abs(wristTargetPosition - (double)wristTickCount()) <= 400){ //100 //deadband for position of wrist
      return true;
    }
    return false;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
