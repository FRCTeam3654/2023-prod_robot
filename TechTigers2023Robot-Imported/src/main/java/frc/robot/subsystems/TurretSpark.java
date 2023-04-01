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

  private static final int deviceID = 15;
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidTurretController;
  private RelativeEncoder m_encoder;
  public double kTurretP, kTurretI, kTurretD, kTurretIz, kTurretFF, kTurretMaxOutput, kTurretMinOutput;
  //private double setPoint;
  private double joystickX;
  private double joysticky;
  public double maxRPM;
  public double holdRotations;



  public TurretSpark() {
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_pidTurretController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    /*kTurretP = 0.1; 
    kTurretI = 1e-4;
    kTurretD = 1; 
    kTurretIz = 0; 
    kTurretFF = 0; 
    kTurretMinOutput = -0.3;
    kTurretMaxOutput = 0.3; 
*/
    kTurretP = 0.3;  //6e-5 //make larger if it doesn't hold //0.1
    kTurretI = 0;
    kTurretD = 1;//0; 
    kTurretIz = 0; 
    kTurretFF = 0.000156;//  0.000015; 
    kTurretMinOutput = -1;
    kTurretMaxOutput = 1; 
    maxRPM = 5700;

    // set PID coefficients
    m_pidTurretController.setP(kTurretP);
    m_pidTurretController.setI(kTurretI);
    m_pidTurretController.setD(kTurretD);
    m_pidTurretController.setIZone(kTurretIz);
    m_pidTurretController.setFF(kTurretFF);
    //m_pidTurretController.setOutputRange(kTurretMinOutput, kTurretMaxOutput);

    kTurretMinOutput = -0.3;
    kTurretMaxOutput = 0.3; 
    m_pidTurretController.setOutputRange(kTurretMinOutput, kTurretMaxOutput); 


    //smart motion setting, similar to Falcon motion magic
    m_pidTurretController.setSmartMotionMaxVelocity(1500, 0);
    m_pidTurretController.setSmartMotionMaxAccel(1500,0);
    m_pidTurretController.setSmartMotionMinOutputVelocity(0, 0);
    m_pidTurretController.setSmartMotionAllowedClosedLoopError(0.1, 0);  

    holdRotations = getSensorReading();

  }

  public void manualTurretControl(double setPoint){

    //setPoint = .2 * maxRPM;
    //setPoint = -0.2 * maxRPM;
    //System.out.println("sensor reading = " + getSensorReading());
    double currrentReading = getSensorReading();
    if( currrentReading > 0 && currrentReading  > 2.88  )  {
      // 2.88 is around turing 90 degree by measuring the number at the robot

      // positive voltage move towards left, if at left 90 degree, cannot  move more, only can move toward right
      if( setPoint < 0) {
        
        m_pidTurretController.setReference(setPoint, CANSparkMax.ControlType.kVoltage);
      }
    }
    else if( currrentReading < 0 && currrentReading  < -2.88  )  {
        // only allow to move to left if at the right most position (90 degree)
        if( setPoint > 0) { 
          m_pidTurretController.setReference(setPoint, CANSparkMax.ControlType.kVoltage);
        }
    }
    else if( Math.abs(currrentReading) < 2.88) {
      m_pidTurretController.setReference(setPoint, CANSparkMax.ControlType.kVoltage);
    }




  } 


  public void resetEncoders(){
    m_encoder.setPosition(0);
  }

  public double getSensorReading(){
    return m_encoder.getPosition();
  }

  public void stayStill(double rotations){
    //CANSparkMax.ControlType.kSmartMotion
    m_pidTurretController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  public void goHome(){
    //m_pidTurretController.setReference(0, CANSparkMax.ControlType.kPosition);
    m_pidTurretController.setReference(0, CANSparkMax.ControlType.kSmartMotion);

  }

  public void goToPosition(double postition){
    m_pidTurretController.setReference(postition, CANSparkMax.ControlType.kPosition);
  }
 
  public void goToPositionBySmartMotion(double postition){
    m_pidTurretController.setReference(postition, CANSparkMax.ControlType.kSmartMotion);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
