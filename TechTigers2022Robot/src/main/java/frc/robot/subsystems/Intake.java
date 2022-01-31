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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;


public class Intake extends SubsystemBase {
  private TalonSRX intakeTalonLeft = new TalonSRX (RobotMap.IntakeTalonLeftID);
  private TalonSRX intakeTalonRight = new TalonSRX (RobotMap.IntakeTalonRightID);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 seeTheRainbow = new ColorSensorV3(i2cPort);
  private final ColorMatch rainbowMatcher = new ColorMatch();
  
  public final Color kBlueTarget = new Color(RobotMap.colorTargets[0][0], RobotMap.colorTargets[0][1], RobotMap.colorTargets[0][2]);
  public final Color kRedTarget = new Color(RobotMap.colorTargets[1][0], RobotMap.colorTargets[1][1], RobotMap.colorTargets[1][2]);

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

    rainbowMatcher.addColorMatch(kBlueTarget);
    rainbowMatcher.addColorMatch(kRedTarget);
  }
  public void intakeWheels(double percentOutput){
    intakeTalonLeft.set(ControlMode.PercentOutput, percentOutput);
    SmartDashboard.putNumber("IntakePercentVoltage", percentOutput);
  }
  public int getRainbow(){
    Color detectedColor = seeTheRainbow.getColor();
    // Run the color match algorithm on our detected color
    String colorString;
    ColorMatchResult match = rainbowMatcher.matchClosestColor(detectedColor);
    int Rainbow;
      if (match.color == kBlueTarget) {
        colorString = "Blue";
        Rainbow = 1;
      } else if (match.color == kRedTarget) {
        colorString = "Red";
        Rainbow = 2;
      
      } else {
        colorString = "Unknown";
        Rainbow = 0;
      }
      if (match.confidence <= 0.95){
        colorString = "Unknown";
        Rainbow = 0;
      }
      //Blue = 1 Red = 2 Green = 3 Yellow = 4
      // Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Confidence", match.confidence);
      SmartDashboard.putString("Detected Color", colorString);
      return Rainbow;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
