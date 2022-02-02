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

public class Beltcro extends SubsystemBase {
  private TalonSRX beltcroTalon = new TalonSRX (RobotMap.BeltcroTalonID);
  /** Creates a new Beltcro. */
  public Beltcro() {
    beltcroTalon.configFactoryDefault();
    beltcroTalon.set(ControlMode.PercentOutput, 0);
    beltcroTalon.setNeutralMode(NeutralMode.Brake);

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
