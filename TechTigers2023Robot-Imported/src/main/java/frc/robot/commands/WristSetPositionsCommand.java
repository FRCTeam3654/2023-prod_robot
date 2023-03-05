// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;


public class WristSetPositionsCommand extends CommandBase {
  /** Creates a new WristSetPositionsCommand. */
  private boolean isDeployButtonPressed = false;
  private boolean wasDeployButtonNotPressed = false;
  private boolean isDownUpButtonPressed = false;
  private boolean wasDownUpButtonNotPressed = false;
  public double wristTimer = 0;
  private static int fullButtonPressNumber = 0;
  private static int deployButtonPressNumber = 0;
  private boolean isLockButtonPressed = false;
  private int lockButtonPressNumber = 0;

  public WristSetPositionsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    fullButtonPressNumber = 0;
    //deployButtonPressNumber = 0;
    lockButtonPressNumber = 0;

    /*if(RobotContainer.oi.wristDeployButton.getAsBoolean() == true){
      isDeployButtonPressed = true;
    }
    else if(RobotContainer.oi.wristDownUpButton.getAsBoolean() == true){
      isDownUpButtonPressed = true;
    }

    wasDownUpButtonNotPressed = false;
    wasDeployButtonNotPressed = false;
    */
    RobotContainer.wrist.zeroSensor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.oi.wristLockButton.getAsBoolean() == true){
      isLockButtonPressed = true;
     }
  
     if(isLockButtonPressed == true){
      RobotContainer.wrist.manualwrist(0);
     }

    //if(isDeployButtonPressed == true || isDownUpButtonPressed == true){
//System.out.println("am i stopped");
    //}

   else{
    //pressing the button will add +1 to the integer
   // if(RobotContainer.oi.wristDeployButton.getAsBoolean() == true){
     // deployButtonPressNumber = deployButtonPressNumber + 1;
      //isDeployButtonPressed = true;
    //}

    if(RobotContainer.oi.wristDownUpButton.getAsBoolean() == true){
      fullButtonPressNumber = fullButtonPressNumber + 1;
      //isDownUpButtonPressed = true;
    }

//lock button

   
//----------------------------------------------------------------------------------------------------

    //if the number is odd, it goes the distance down; if the number is even, bring the arm up
    if(fullButtonPressNumber % 2 == 1){
      RobotContainer.wrist.setMotionMagic(RobotMap.wristFullUpDistance, 2000, 2000);
      isLockButtonPressed = false;
      //System.out.println("should i be moving down");
      wristTimer = Timer.getFPGATimestamp();
    }

    else if(fullButtonPressNumber % 2 == 0){
      RobotContainer.wrist.setMotionMagic(-1 * RobotMap.wristFullUpDistance, 2000, 2000);
      isLockButtonPressed = false;
      //System.out.println("should i be moving up");
      wristTimer = Timer.getFPGATimestamp();
    }

    /* 
    if(deployButtonPressNumber % 2 == 1){
      RobotContainer.wrist.setMotionMagic(RobotMap.wristDeployDistance, 1000, 1000);
      isLockButtonPressed = false;
      System.out.println("should i be moving deploy down");
      wristTimer = Timer.getFPGATimestamp();
    }

    else if(deployButtonPressNumber % 2 == 0){
      RobotContainer.wrist.setMotionMagic(-1 * RobotMap.wristDeployDistance, 1000, 1000);
      isLockButtonPressed = false;
      System.out.println("should i be moving deploy up");
      wristTimer = Timer.getFPGATimestamp();
    }
    */
    //end of the odd/even functions

//-------------------------------------------------------------------------------------------------------------------

    //if the wrist is fully down and the wrist deploy button is pressed, nothing happens and the deploy number stays the same
    if(fullButtonPressNumber % 2 == 1 && RobotContainer.oi.wristDeployButton.getAsBoolean() == true){
      RobotContainer.wrist.setMotionMagic(0, 0, 0);
      deployButtonPressNumber = deployButtonPressNumber - 1;
      //System.out.println("why am i stopped");

    }

    //if the wrist is in deploy position and the full distance button is pressed, nothing happens and the full press number stays the same
    //if(deployButtonPressNumber % 2 == 1 && RobotContainer.oi.wristDownUpButton.getAsBoolean() == true){
      //RobotContainer.wrist.setMotionMagic(0, 0, 0);
      //fullButtonPressNumber = fullButtonPressNumber - 1;
      //System.out.println("why am i stopped");
    }
   //------------------------------------------------------------------------------------------------------------------------- 



    /*
    isDownUpButtonPressed=RobotContainer.oi.wristDownUpButton.getAsBoolean();
    isDeployButtonPressed=RobotContainer.oi.wristDeployButton.getAsBoolean();

    //if the down/up button is pressed and lifted up, was button not pressed is true
    if (RobotContainer.oi.wristDownUpButton.getAsBoolean()){
      if (!isDownUpButtonPressed) {
        wasDownUpButtonNotPressed=true;
      }
    }

    //if the deploy button is pressed and lifted up, was button not pressed is true
    if (RobotContainer.oi.wristDeployButton.getAsBoolean()){
      if (!isDeployButtonPressed) {
        wasDeployButtonNotPressed=true;
      }
    }

    //if the button has been lifted up it runs the motion magic
    if(!RobotContainer.oi.wristDownUpButton.getAsBoolean() && wasDownUpButtonNotPressed == true){
      RobotContainer.wrist.setMotionMagic(RobotMap.wristFullUpDistance, 500, 500);

      //if the button is pressed again, it runs the opposite motion magic
      if (RobotContainer.oi.wristDownUpButton.getAsBoolean() && wasDownUpButtonNotPressed == true){
        RobotContainer.wrist.setMotionMagic(-1 * RobotMap.wristFullUpDistance, 500, 500);
        isDownUpButtonPressed = true;
      }
    }

    //if the 
    else if (!RobotContainer.oi.wristDeployButton.getAsBoolean() && isDeployButtonPressed == true){
      RobotContainer.wrist.setMotionMagic(RobotMap.wristDeployDistance, 500, 500);

      //if the button is pressed again, it runs the opposite motion magic
      if (RobotContainer.oi.wristDeployButton.getAsBoolean() && wasDeployButtonNotPressed == true){
        RobotContainer.wrist.setMotionMagic(-1 * RobotMap.wristDeployDistance, 500, 500);
        isDeployButtonPressed = true;
      }
    }
    */
 
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //isDeployButtonPressed = false;
    isDownUpButtonPressed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (wristTimer + 3.0) < Timer.getFPGATimestamp()) {
      // after 3 second, stop command
      return true;
    }
    else {
      if (isDownUpButtonPressed){
        double sensorDistance = Math.abs(RobotContainer.wrist.getWristTalonPosition());
        double percentError = 100 * (RobotMap.wristFullUpDistance - sensorDistance)/RobotMap.wristFullUpDistance;

        if (Math.abs(percentError) < 1){
        //if (percentLeftError < 0.9 || percentLeftError < 0 )
        return true;
      }
      }

      /*else if (isDeployButtonPressed){
        double sensorDistance = Math.abs(RobotContainer.wrist.getWristTalonPosition());
        double percentError = 100 * (RobotMap.wristDeployDistance - sensorDistance)/RobotMap.wristDeployDistance;

        if (Math.abs(percentError) < 1){
        //if (percentLeftError < 0.9 || percentLeftError < 0 )
        return true;
        
      }
      }
      */

  }
  return false;
}
}
