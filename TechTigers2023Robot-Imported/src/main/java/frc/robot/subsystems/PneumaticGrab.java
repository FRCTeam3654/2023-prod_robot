// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.CompressorConfigType;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import frc.robot.RobotContainer;
//import com.revrobotics.REVLibError;
//import com.revrobotics.AnalogInput;
//import edu.wpi.first.wpilibj.AnalogOutput;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.PneumaticHub;

/* THis is an alternate method that was used in demo software Andy K

  import edu.wpi.first.wpilibj.PneumaticHub;
  private static final int PH_CAN_ID = 5;
  PneumaticHub m_ph = new PneumaticHub(PH_CAN_ID);
  m_ph.enableCompressorAnalog(minPressure,maxPressure);
*/

public class PneumaticGrab extends SubsystemBase {
  /** Creates a new PneumaticGrab. */
  private Compressor emmasCompressor = new Compressor (5, PneumaticsModuleType.REVPH);
  private DoubleSolenoid emmasSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0,1);

  public PneumaticGrab() {
    // emmasCompressor.enableDigital();   // Removed by andy
    emmasCompressor.enableAnalog(100, 120);  //Added by Andy
    emmasCompressor.isEnabled();
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
