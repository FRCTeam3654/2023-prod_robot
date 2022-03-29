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
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallStorage extends SubsystemBase {
  /** Creates a new BallStorage. */
  public int ballCounter = 3;
  private AnalogInput analogDistanceSensor1;
  private DigitalInput digitalDistanceSensor2;
  private DigitalInput digitalDistanceSensor3;
  //private TalonSRX ballStorageMotor = new //right here there needs to be the type of motor//(RobotMap.BallStorageMotorID); //bottom motor of belt, neg value to move belt forward
  public BallStorage() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
