/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Turn90DegreesCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.BeltcroShooterCommand;
import frc.robot.commands.IntakeOverrideCommand;
import frc.robot.commands.SlidingClimbHooksCommand;
import frc.robot.commands.SlidingClimbManualControlCommand;
import frc.robot.commands.VerticalClimbCommand;
import frc.robot.commands.VerticalClimbMotionMagicUpCommand;
import frc.robot.commands.BallShooterCommand;
//import frc.robot.commands.AutonomousDriveCommand;
//import frc.robot.commands.DriveTargetCommand;
//import frc.robot.commands.BallFlushCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  public Joystick driverStick = new Joystick(RobotMap.driverJoystickPort);
  public Joystick operatorStick = new Joystick(RobotMap.operatorJoystickPort);
  public JoystickButton turboButton;
  public JoystickButton driveStraightButton;
  public JoystickButton turnLeft90Button;
  public JoystickButton turnRight90Button; 
  public JoystickButton turnLeft180Button;
  public JoystickButton turnRight180Button;
  public JoystickButton intakeStopButton;
  public JoystickButton slidingClimbButton;
  public JoystickButton verticalClimbDownButton;
  public JoystickButton verticalClimbUpButton;
  public JoystickButton beltcroShooterButton;
  public JoystickButton manualSlidingClimbButton;
  public JoystickButton slidingClimbReverseButton;
  public JoystickButton slidingTraversalDownButton;
  public JoystickButton climbLockButton;
  public JoystickButton beltcroReverseButton;

  public JoystickButton slidingShortClimbButton;
  public JoystickButton slidingShortClimbReverseButton;
 

  public JoystickButton limeLightButton;
  public JoystickButton climbLockLeftButton;
  public JoystickButton climbLockRightButton;
  public JoystickButton climbUnlockLeftButton;
  public JoystickButton climbUnlockRightButton;
  public JoystickButton intakeOverrideButton;
  public JoystickButton ballShooterButton;
  public JoystickButton highGoalShooterButton;
  //public JoystickButton turretButton;

  public OI(){

  turboButton = new JoystickButton(driverStick, RobotMap.turboButtonNumber);
  driveStraightButton = new JoystickButton(driverStick, RobotMap.driveStraightButtonNumber);
  turnLeft90Button = new JoystickButton(driverStick, RobotMap.turnLeft90ButtonNumber);
  turnRight90Button = new JoystickButton(driverStick, RobotMap.turnRight90ButtonNumber);
  turnLeft180Button = new JoystickButton(driverStick, RobotMap.turnLeft180ButtonNumber);
  turnRight180Button = new JoystickButton(driverStick, RobotMap.turnRight180ButtonNumber);
  intakeStopButton = new JoystickButton(operatorStick, RobotMap.intakeStopButtonNumber);
  manualSlidingClimbButton = new JoystickButton(driverStick, RobotMap.manualSlidingClimbButtonNumber);
  verticalClimbUpButton = new JoystickButton(operatorStick, RobotMap.verticalClimbUpButtonNumber);
  verticalClimbDownButton = new JoystickButton(operatorStick, RobotMap.verticalClimbDownButtonNumber); //change to operatorStick when we have both joystick
  beltcroShooterButton = new JoystickButton(operatorStick, RobotMap.beltcroShooterButtonNumber); //change to operatorStick when we have both joystick
  intakeOverrideButton = new JoystickButton(operatorStick, RobotMap.intakeOverrideButtonNumber);
  //limeLightButton = new JoystickButton(driverStick, RobotMap.limeLightButtonNumber);
  slidingClimbButton = new JoystickButton(driverStick, RobotMap.slidingClimbButtonNumber);
  slidingClimbReverseButton = new JoystickButton(driverStick, RobotMap.slidingClimbReverseButtonNumber);
  slidingTraversalDownButton = new JoystickButton(driverStick, RobotMap.slidingTraversalDownButtonNumber);
  climbLockButton = new JoystickButton(operatorStick, RobotMap.climbLockButtonNumber);
  beltcroReverseButton = new JoystickButton(operatorStick, RobotMap.beltcroReverseButtonNumber);
  ballShooterButton = new JoystickButton(operatorStick, RobotMap.ballShooterButtonNumber);
  highGoalShooterButton = new JoystickButton(operatorStick, RobotMap.highGoalShooterButtonNumber);

  slidingShortClimbButton = new JoystickButton(operatorStick, RobotMap.slidingShortClimbButtonNumber);
  slidingShortClimbReverseButton = new JoystickButton(operatorStick, RobotMap.slidingShortClimbReverseButtonNumber);

  //climbLockRightButton = new JoystickButton(operatorStick, RobotMap.climbLockRightButtonNumber);
  //climbLockLeftButton = new JoystickButton(operatorStick, RobotMap.climbLockRightButtonNumber);
  //climbUnlockLeftButton = new JoystickButton(operatorStick, RobotMap.climbUnlockLeftButtonNumber);
  //climbUnlockRightButton = new JoystickButton(operatorStick, RobotMap.climbUnlockRightButtonNumber);
  //turretButton = new JoystickButton(operatorStick, RobotMap.turretButtonNumber);

  turnLeft90Button.whenPressed(new Turn90DegreesCommand());
  turnRight90Button.whenPressed(new Turn90DegreesCommand());
  turnLeft180Button.whenPressed(new Turn90DegreesCommand());
  turnRight180Button.whenPressed(new Turn90DegreesCommand());
  intakeStopButton.whenPressed(new IntakeStopCommand());
  beltcroShooterButton.whenPressed(new BeltcroShooterCommand());
  slidingClimbButton.whenPressed(new SlidingClimbHooksCommand());
  slidingClimbReverseButton.whenPressed(new SlidingClimbHooksCommand());
  slidingTraversalDownButton.whenPressed(new SlidingClimbHooksCommand(3));
  manualSlidingClimbButton.whenPressed(new SlidingClimbManualControlCommand());
  verticalClimbUpButton.whenPressed(new VerticalClimbMotionMagicUpCommand());
  verticalClimbDownButton.whenPressed(new VerticalClimbMotionMagicUpCommand());
  climbLockButton.whenPressed(new VerticalClimbCommand());
  beltcroReverseButton.whenPressed(new BeltcroShooterCommand());
  ballShooterButton.whenPressed(new BallShooterCommand(0, 0));
  highGoalShooterButton.whenPressed(new BallShooterCommand(2, 1));
  intakeOverrideButton.whenPressed(new IntakeOverrideCommand());

  slidingShortClimbButton.whenPressed(new SlidingClimbHooksCommand(RobotMap.slidingShortClimbDistance));
  slidingShortClimbReverseButton.whenPressed(new SlidingClimbHooksCommand(RobotMap.slidingShortClimbDistance));


  }
  
}
