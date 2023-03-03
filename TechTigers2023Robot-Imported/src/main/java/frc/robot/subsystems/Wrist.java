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
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/*
Emily we might want to add some of these two things somewere in this  subsystem

TalonSRX talon = new TalonSRX(0);
talon.configPeakCurrentLimit(30); // don't activate current limit until current exceeds 30 A ...
talon.configPeakCurrentDuration(100); // ... for at least 100 ms
talon.configContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
talon.enableCurrentLimit(true);



 Configured forward and reverse limit switch of Talon to be from a feedback connector and be normally open 
Hardware.leftTalonMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
Hardware.leftTalonMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
Limit Switch Override Enable
The enable state of the limit switches can be overridden in software. This can be called at any time to enable or disable both limit switches.

Generally you should call this instead of a config if you want to dynamically change whether you are using the limit switch or not inside a loop. This value is not persistent across power cycles.

 Limit switches are forced disabled on Talon and forced enabled on Victor 
Hardware.leftTalonMaster.overrideLimitSwitchesEnable(false);
Hardware.rightVictorMaster.overrideLimitSwitchesEnable(true);;



Soft Limits
Soft limits can be used to disable motor drive when the “Sensor Position” is outside of a specified range. Forward throttle will be disabled if the “Sensor Position” is greater than the Forward Soft Limit. Reverse throttle will be disabled if the “Sensor Position” is less than the Reverse Soft Limit. The respective Soft Limit Enable must be enabled for this feature to take effect.

 Talon configured to have soft limits 10000 native units in either direction and enabled 
rightMaster.configForwardSoftLimitThreshold(10000, 0);
rightMaster.configReverseSoftLimitThreshold(-10000, 0);
rightMaster.configForwardSoftLimitEnable(true, 0);
rightMaster.configReverseSoftLimitEnable(true, 0);

var forwardLimit = m_motor.getForwardLimit();

if (forwardLimit.getValue() == ForwardLimitValue.ClosedToGround) {
   // do action when forward limit is closed
}



*/

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private WPI_TalonFX wristTalon = new WPI_TalonFX (RobotMap.wristTalonID);
  //private TalonFX wristTalon = new TalonFX (RobotMap.wristTalonID);

  private double wristTargetPosition;
  
  public Wrist() {
    wristTalon.configFactoryDefault();
    
    
    wristTalon.setNeutralMode(NeutralMode.Brake);
    wristTalon.configNeutralDeadband(0.01, RobotMap.pidLoopTimeout); //during testing was 0.001

    wristTalon.configClosedloopRamp(1);
    wristTalon.configOpenloopRamp(1);
    
    wristTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);

    /* Set relevant frame periods to be at least as fast as periodic rate */
		wristTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.pidLoopTimeout);
		wristTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.pidLoopTimeout);

		/* Set the peak and nominal outputs */
		wristTalon.configNominalOutputForward(0, 30);
		wristTalon.configNominalOutputReverse(0, 30);
		wristTalon.configPeakOutputForward(1, 30);
		wristTalon.configPeakOutputReverse(-1, 30);

    wristTalon.config_kF(0,0.045,30); // 0.045
    wristTalon.config_kP(0,0.049,30); //0.095 //0.049
    wristTalon.config_kI(0,0,30);
    wristTalon.config_kD(0,0,30);

		/* Set gains in slot0 - see documentation */
    //wristTalon.selectProfileSlot(RobotMap.kShooterSlotIDx, RobotMap.kPIDLoopIDx);
		//wristTalon.config_kF(RobotMap.kShooterSlotIDx, RobotMap.wristGainsVelocity.kF, RobotMap.pidLoopTimeout);
		//wristTalon.config_kP(RobotMap.kShooterSlotIDx, RobotMap.wristGainsVelocity.kP, RobotMap.pidLoopTimeout);
		//wristTalon.config_kI(RobotMap.kShooterSlotIDx, RobotMap.wristGainsVelocity.kI, RobotMap.pidLoopTimeout);
		//wristTalon.config_kD(RobotMap.kShooterSlotIDx, RobotMap.wristGainsVelocity.kD, RobotMap.pidLoopTimeout);

		/* Set acceleration and vcruise velocity - see documentation */
		wristTalon.configMotionCruiseVelocity(2000, RobotMap.pidLoopTimeout);
		wristTalon.configMotionAcceleration(2000, RobotMap.pidLoopTimeout);

		/* Zero the sensor once on robot boot up */    
    zeroSensor();
  }

  public void zeroSensor() {
    wristTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    //System.out.println("wrist Sensor is 0");
  }

  public double getWristTalonPosition(){
    return wristTalon.getSelectedSensorPosition(0);
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
    wristTalon.enableVoltageCompensation(true);
    wristTalon.configVoltageCompSaturation(11, 0);
    wristTalon.set(ControlMode.PercentOutput, percentOutput);
  }
  public void setMotionMagic(double distance, int cruiseVelocity, int accelerationVelocity ) {
    setMotionMagic( distance, cruiseVelocity,  accelerationVelocity, 0) ;
   }


  public void setMotionMagic(double distance, int cruiseVelocity, int accelerationVelocity, double arbitraryFeedForwardValue) {
        wristTalon.configMotionCruiseVelocity(cruiseVelocity, RobotMap.pidLoopTimeout);
        wristTalon.configMotionAcceleration(accelerationVelocity, RobotMap.pidLoopTimeout);
      
        //wristTalon.configMotionAcceleration(accelerationVelocity, RobotMap.pidLoopTimeout);
      
        //wristTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);

        if( Math.abs(arbitraryFeedForwardValue) > 0.001) {
          // add  DemandType.ArbitraryFeedForward to hold the vertical arm in position
          wristTalon.set(ControlMode.MotionMagic, distance,  DemandType.ArbitraryFeedForward, arbitraryFeedForwardValue);
        }
        else {
          wristTalon.set(ControlMode.MotionMagic, distance);
        }
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
