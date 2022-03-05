// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.RobotState;
//import frc.robot.Robot;
//import frc.robot.RobotMap;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class VerticalClimbCommand extends CommandBase {
  /** Creates a new VerticalClimbCommand. */
  public VerticalClimbCommand() {
    addRequirements(RobotContainer.verticalClimbArms);
  // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //sets the z channel for climb on xbox joystick so it works
    RobotContainer.oi.operatorStick.setZChannel(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickX;
    double joystickZ;
    //joystickX = (RobotContainer.oi.operatorStick.getY());
    //joystickZ = (RobotContainer.oi.operatorStick.getZ());
    
    joystickX = (RobotContainer.oi.operatorStick.getY());
    joystickZ = (RobotContainer.oi.operatorStick.getX());
  

    //joystickX = handleDeadband(joystickX, RobotMap.joystickDeadBand);
    //joystickY = handleDeadband(joystickY, RobotMap.joystickDeadBand);
    
    if (Math.abs(joystickX) < RobotMap.joystickDeadBand)
    {
      joystickX = 0;
    }
    if (Math.abs(joystickZ) < RobotMap.joystickDeadBand)
    {
      joystickZ = 0;
    }

    if (joystickX < 0)
    {
      joystickX *= 0.5;
    }
    if (joystickZ < 0)
    {
      joystickZ *= 0.5;
    }

    //joystickZ = 0; //reomove this for actual operator controller controls

    SmartDashboard.putNumber("JoystickX", joystickX);
    SmartDashboard.putNumber("JoystickZ", joystickZ);

    RobotContainer.verticalClimbArms.karenaArcadeDrive(joystickZ, joystickX);
    //RobotContainer.verticalClimbArms.karenaNotArcadeDrive(joystickZ, joystickX);
     
    System.out.println("ElevatorX = " + joystickX + "ElevatorZ = " + joystickZ);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
