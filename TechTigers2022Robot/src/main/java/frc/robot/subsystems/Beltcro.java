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
//import com.ctre.phoenix.motorcontrol.InvertType; 
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DigitalInput;

public class Beltcro extends SubsystemBase {
  private TalonSRX beltcroTalon = new TalonSRX (RobotMap.BeltcroTalonID);
  /** Creates a new Beltcro. */
  public int ballCounter = 3;
  private AnalogInput analogDistanceSensor1;
  public Beltcro() {
    analogDistanceSensor1 = new AnalogInput(RobotMap.analogDistanceSensorPort1);
    analogDistanceSensor1.setAverageBits(12);

    beltcroTalon.configFactoryDefault();
    beltcroTalon.set(ControlMode.PercentOutput, 0);
    beltcroTalon.setNeutralMode(NeutralMode.Brake);

  }
  public double storageSensor1() {
    //double cmDistanceSensor = (27048/(analogDistanceSensor.getAverageValue()-36))-4;
    double cmDistanceSensor1;
    cmDistanceSensor1 = analogDistanceSensor1.getAverageValue();
    SmartDashboard.putNumber("Analog Distance Sensor 1 raw", cmDistanceSensor1);
    return cmDistanceSensor1;
  }

  public void beltcroMove(double percentOutput){
    beltcroTalon.set(ControlMode.PercentOutput, percentOutput);
    SmartDashboard.putNumber("BeltcroPercentVoltage", percentOutput);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
