// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;

public class PneumaticsGrabbingCommand extends CommandBase {
  /** Creates a new PneumaticsGrabbingCommand. */
  private int mode = 0; //0 is regular, 1 is auto open, 2 is auto close
  private double pneumaticTimer;
  public PneumaticsGrabbingCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pneumaticGrab);
  }
  public PneumaticsGrabbingCommand(int new_mode){
    addRequirements(RobotContainer.pneumaticGrab);
    mode = new_mode;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

     if(RobotContainer.oi.operatorStick.getRightTriggerAxis() > 0.4 || mode == 2){ // opens
      RobotContainer.pneumaticGrab.practiceSolenoid(true);
      //pneumaticTimer = Timer.getFPGATimestamp();
    }

    else if(RobotContainer.oi.operatorStick.getLeftTriggerAxis() > 0.4 || mode == 1){ //closes
      RobotContainer.pneumaticGrab.practiceSolenoid(false);
      //pneumaticTimer = Timer.getFPGATimestamp();
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      //if( (pneumaticTimer + 1.0) < Timer.getFPGATimestamp()) {
        // after 3 second, stop command
        //return true;
      //}

    /*if (RobotContainer.oi.pneumaticGrabButton.getAsBoolean() == false) {
      RobotContainer.pneumaticGrab.practiceSolenoid(false);
      return true;
    }
    */
    return false;
  }
}
