// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CompressorConfigType;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import frc.robot.RobotContainer;
//import com.revrobotics.REVLibError;
import com.revrobotics.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.PneumaticHub;

 //THis is an alternate method that was used in demo software Andy K

  import edu.wpi.first.wpilibj.PneumaticHub;



public class PneumaticGrab extends SubsystemBase {
  /** Creates a new PneumaticGrab. */
  //private Compressor emmasCompressor = new Compressor (5, PneumaticsModuleType.REVPH);
  
  private static final int PH_CAN_ID = 5;
  private static int forwardChannel = 0;
  private static int reverseChannel = 1;
  PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);
  DoubleSolenoid m_doubleSolenoid = m_pH.makeDoubleSolenoid(forwardChannel, reverseChannel);
  //private DoubleSolenoid emmasSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0,1);


  public PneumaticGrab() {
    // emmasCompressor.enableDigital();   // Removed by andy
    //emmasCompressor.enableAnalog(100, 120);  //Added by Andy
    //emmasCompressor.isEnabled();
    m_pH.enableCompressorAnalog(100, 120);

  }
  public void practiceSolenoid(boolean onOff){
    if(m_doubleSolenoid == null){
      //System.out.println("emmas solenoid is null");
      return;
    }
    else{
      //System.out.println("not null");
  }
    if (onOff == true){
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
      //System.out.println("am i supposed to be expanding");
    }

    else if (onOff == false){
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      //System.out.println("am i supposed to be contracting");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
