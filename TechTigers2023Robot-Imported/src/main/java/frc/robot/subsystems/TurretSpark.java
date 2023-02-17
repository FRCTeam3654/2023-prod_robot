// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class TurretSpark extends SubsystemBase {
  /** Creates a new TurretSpark. */

  private static final int deviceID = 1;
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidTurretController;
  private RelativeEncoder m_encoder;
  public double kTurretP, kTurretI, kTurretD, kTurretIz, kTurretFF, kTurretMaxOutput, kTurretMinOutput;
  private double setPoint;

  public TurretSpark() {
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_pidTurretController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kTurretP = 0.1; 
    kTurretI = 1e-4;
    kTurretD = 1; 
    kTurretIz = 0; 
    kTurretFF = 0; 
    kTurretMaxOutput = 1; 
    kTurretMinOutput = -1;

    // set PID coefficients
    m_pidTurretController.setP(kTurretP);
    m_pidTurretController.setI(kTurretI);
    m_pidTurretController.setD(kTurretD);
    m_pidTurretController.setIZone(kTurretIz);
    m_pidTurretController.setFF(kTurretFF);
    m_pidTurretController.setOutputRange(kTurretMinOutput, kTurretMaxOutput);

    kTurretMinOutput = -0.3;
    kTurretMaxOutput = 0.3; 
    m_pidTurretController.setOutputRange(kTurretMinOutput, kTurretMaxOutput); 



  }

  public void manualTurretControl(double setPoint){
    m_pidTurretController.setReference(setPoint, CANSparkMax.ControlType.kVoltage);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
