// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;



public class PneumaticsTesting extends SubsystemBase {
  private Compressor emmasCompressor = new Compressor (0, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid emmasSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0,1);
  /** Creates a new EmmaPneumatics. */
  public PneumaticsTesting() {
    emmasCompressor.enableDigital();
    emmasCompressor.enabled();
  }
  public void practiceSolenoid(boolean onOff){
    if (onOff){
      emmasSolenoid.set(DoubleSolenoid.Value.kForward);
    }

  else{
    emmasSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
