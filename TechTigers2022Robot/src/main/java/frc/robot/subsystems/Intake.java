// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;


public class Intake extends SubsystemBase {
  private TalonSRX intakeTalonLeft = new TalonSRX (RobotMap.IntakeTalonLeftID);
  private TalonSRX intakeTalonRight = new TalonSRX (RobotMap.IntakeTalonRightID);

  /** Creates a new Intake. */
  public Intake() {
    intakeTalonLeft.configFactoryDefault();
    intakeTalonRight.configFactoryDefault();

    intakeTalonLeft.set(ControlMode.PercentOutput, 0);
    intakeTalonRight.set(ControlMode.PercentOutput, 0);
    
    intakeTalonRight.follow(intakeTalonLeft);

    intakeTalonLeft.setInverted(false);
    intakeTalonRight.setInverted(true);
    //need to invert one of the sides but are undecided on which side
    intakeTalonLeft.setNeutralMode(NeutralMode.Coast);
    intakeTalonRight.setNeutralMode(NeutralMode.Coast);
    //possible to set ramp rate
  }
  public void intakeWheels(double percentOutput){
    intakeTalonLeft.set(ControlMode.PercentOutput, percentOutput);
    SmartDashboard.putNumber("IntakePercentVoltage", percentOutput);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
