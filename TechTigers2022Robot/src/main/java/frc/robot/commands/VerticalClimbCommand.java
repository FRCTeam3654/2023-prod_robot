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
  //private boolean IsLockButtonPressed=false;
  //private boolean isLocked = false;  // need be static or not ??
  //public static boolean isSlidingClimberMovingDown = false;  // let SlidingClimber to tell it is moving down

  /** Creates a new VerticalClimbCommand. */
  public VerticalClimbCommand() {
    addRequirements(RobotContainer.verticalClimbArms);
  // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //IsLockButtonPressed=false;
    //isLocked = false;
    //sets the z channel for climb on xbox joystick so it works
    //RobotContainer.oi.operatorStick.setZChannel(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
      boolean IsLockButtonPressed=RobotContainer.oi.climbLockButton.get();
      
      if (!IsLockButtonPressed) {
        WasLockButtonNotPressed=true;
      }
      else {
        WasLockButtonNotPressed = false;
      }
      */
      /*
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
      
      SmartDashboard.putBoolean("isLocked_VerticalClimb", isLocked);
      SmartDashboard.putBoolean("IsLockButtonPressed_VerticalClimb", IsLockButtonPressed);
     */
    
      double joystickX;
      double joystickY;
     
      joystickY = (RobotContainer.oi.operatorStick.getY());
      joystickX = (RobotContainer.oi.operatorStick.getX());
    
      SmartDashboard.putNumber("JoystickX", joystickX);
      SmartDashboard.putNumber("JoystickY", joystickY);

      /*
      // add additional logic to unlock:  1) by button 2) by joystick move up ) sliding climber
      if(Math.abs(joystickY) > 0.25  || isSlidingClimberMovingDown == true) {
        // reset the lock condition
        isLocked = false;
        IsLockButtonPressed = false;// reset
      }
      */

      if(Math.abs(joystickY) > 0.25 ) {
        // reset the lock condition
        VerticalClimbHoldCommand.cancelLock = true;
      }


      //if ( isLocked == true ) {
      //  RobotContainer.verticalClimbArms.sizzleClimbHold();
      //}
      //else {
       

        RobotContainer.verticalClimbArms.karenaArcadeDrive(joystickX, joystickY);
          
        //System.out.println("ElevatorX = " + joystickX + "ElevatorY = " + joystickY);

      //}
    
}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (IsLockButtonPressed && WasLockButtonNotPressed){
    //  return true;
    //}
    return false;
  }
}
