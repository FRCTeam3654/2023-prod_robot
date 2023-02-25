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
import com.ctre.phoenix.motorcontrol.can.TalonFX;


public class VerticalMotionArm extends SubsystemBase {
  /** Creates a new VerticalMotionArm. */

  private TalonFX verticalArmTalon = new TalonFX (RobotMap.armVerticalTalonID);
  private double verticalArmTargetPosition;

  public VerticalMotionArm() {
    verticalArmTalon.configFactoryDefault();
    
    
    verticalArmTalon.setNeutralMode(NeutralMode.Brake);
    verticalArmTalon.configNeutralDeadband(0.01, RobotMap.pidLoopTimeout); //during testing was 0.001
    
    verticalArmTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);

    /* Set relevant frame periods to be at least as fast as periodic rate */
		verticalArmTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.pidLoopTimeout);
		verticalArmTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.pidLoopTimeout);

		/* Set the peak and nominal outputs */
		verticalArmTalon.configNominalOutputForward(0, RobotMap.pidLoopTimeout);
		verticalArmTalon.configNominalOutputReverse(0, RobotMap.pidLoopTimeout);
		verticalArmTalon.configPeakOutputForward(1, RobotMap.pidLoopTimeout);
		verticalArmTalon.configPeakOutputReverse(-1, RobotMap.pidLoopTimeout);

		/* Set gains in slot0 - see documentation */
    verticalArmTalon.selectProfileSlot(RobotMap.kShooterSlotIDx, RobotMap.kPIDLoopIDx);
		verticalArmTalon.config_kF(RobotMap.kShooterSlotIDx, RobotMap.verticalArmGainsVelocity.kF, RobotMap.pidLoopTimeout);
		verticalArmTalon.config_kP(RobotMap.kShooterSlotIDx, RobotMap.verticalArmGainsVelocity.kP, RobotMap.pidLoopTimeout);
		verticalArmTalon.config_kI(RobotMap.kShooterSlotIDx, RobotMap.verticalArmGainsVelocity.kI, RobotMap.pidLoopTimeout);
		verticalArmTalon.config_kD(RobotMap.kShooterSlotIDx, RobotMap.verticalArmGainsVelocity.kD, RobotMap.pidLoopTimeout);

		/* Set acceleration and vcruise velocity - see documentation */
		verticalArmTalon.configMotionCruiseVelocity(500, RobotMap.pidLoopTimeout);
		verticalArmTalon.configMotionAcceleration(500, RobotMap.pidLoopTimeout);

		/* Zero the sensor once on robot boot up */    
    zeroSensor();
  }

  public void zeroSensor() {
    verticalArmTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    //System.out.println("verticalArm Sensor is 0");
  }

  public void verticalArmMovement(double targetPos) {
    if(targetPos > 5000){
      targetPos = 5000;
    }
    if(targetPos < -5000){
      targetPos = -5000;  //adjust 5000 based on experimental limits 
    }
    verticalArmTargetPosition = targetPos;
    verticalArmTalon.set(ControlMode.MotionMagic, targetPos);
    System.out.println(verticalArmTalon.getSelectedSensorVelocity(0) + ",  " + verticalArmTalon.getClosedLoopError(0) + ",  " + verticalArmTalon.getSelectedSensorPosition(0));
  }

  public int verticalArmTickCount(){
    return (int)verticalArmTalon.getSelectedSensorPosition();   
  }

  public void manualVerticalArm(double percentOutput){
    verticalArmTalon.set(ControlMode.PercentOutput, percentOutput);
  }


  public boolean atTargetPosition(){
    if(Math.abs(verticalArmTargetPosition - (double)verticalArmTickCount()) <= 400){ //100 //deadband for position of verticalArm
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}