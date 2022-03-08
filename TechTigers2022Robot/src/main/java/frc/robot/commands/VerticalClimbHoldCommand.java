// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotMap;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class VerticalClimbHoldCommand extends CommandBase {
  private boolean IsLockButtonPressed=false;
  private boolean isLocked = false;  // need be static or not ??
  public static boolean cancelLock = false;  // let SlidingClimber to tell it is moving down

  /** Creates a new VerticalClimbHoldCommand. */
  public VerticalClimbHoldCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.verticalClimbArms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IsLockButtonPressed=false;
    isLocked = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      boolean isLockButtonCurrentlyPressed =RobotContainer.oi.climbLockButton.get();

      if(isLockButtonCurrentlyPressed == true) {
        if( isLocked == false && IsLockButtonPressed == false) {
          // not locking --> press lock button to lock it
          isLocked = true;  
        }
        else if ( isLocked == true && IsLockButtonPressed == false) {
          // if it is currently locked,  unlocked when pressed again (toggle)
          isLocked = false;
        }
        IsLockButtonPressed = true;
      }
      else  {
        if( isLocked == true && IsLockButtonPressed == true) {
          // when just pressed lock button and lock is in action, release lock button will still keep the lock
          isLocked = true;
        }
        else if ( isLocked == false && IsLockButtonPressed == true) {
          // when it is not locked after lock button is pressed (toggled from lock to unlock), release lock button will still keep the unlock
          isLocked = false;
        }

        IsLockButtonPressed = false;
      
      }


      double joystickY = (RobotContainer.oi.operatorStick.getY());
      if ( Math.abs(joystickY) > 0.25 )
      {
        cancelLock = true;
      }

      // add additional logic to unlock:  1) by button 2) by joystick move up ) sliding climber
      if(cancelLock == true ) {
        // reset the lock condition
        isLocked = false;
        IsLockButtonPressed = false;// reset
      }

      SmartDashboard.putBoolean("IsLockedHold_VerticalClimb", isLocked);

      if ( isLocked == true ) {
        RobotContainer.verticalClimbArms.sizzleClimbHold();
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(cancelLock == true) {
       return true;
    }
    return false;
  }
}
