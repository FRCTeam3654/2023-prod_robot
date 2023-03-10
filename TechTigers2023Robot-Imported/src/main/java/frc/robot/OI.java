/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Turn90DegreesCommand;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ManualTurretTurningCommand;
import frc.robot.commands.PneumaticsGrabbingCommand;
import frc.robot.commands.ApriltagTurnCommand;
import frc.robot.commands.ArmSetPositionsCommand;
import frc.robot.commands.ManualWristCommand;
import frc.robot.commands.WristSetPositionsCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.Turn90DegreesCommand;
import frc.robot.commands.WristMotionMagic;
import frc.robot.commands.ArmJoustCommand;
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
  //public Joystick driverStick = new Joystick(RobotMap.driverJoystickPort);
  //public Joystick operatorStick = new Joystick(RobotMap.operatorJoystickPort);

  public XboxController driverStick = new XboxController(RobotMap.driverControllerPort);
  public XboxController operatorStick = new XboxController(RobotMap.operatorControllerPort);
  public Trigger turboButton;
  public JoystickButton driveStraightButton;
  public JoystickButton turnLeft90Button;
  public JoystickButton turnRight90Button; 
  public JoystickButton turnLeft180Button;
  public JoystickButton turnRight180Button;
  public Trigger pneumaticGrabButton;
  public Trigger pneumaticDropButton;
  public JoystickButton balanceButton;
  public POVButton turretRightPOV;
  public POVButton turretLeftPOV;
  public POVButton wristUpPOV;
  public POVButton wristDownPOV;
  public JoystickButton armFullOutButton;
  public JoystickButton armFullBackButton;
  public JoystickButton wristDeployButton;
  public JoystickButton wristDownUpButton;
  //public JoystickButton wristLockButton;
  //public JoystickButton armLockButton;
  public JoystickButton initialPitchButton;
  public JoystickButton photonvisionButton;
  public JoystickButton armPivotButton;
  public JoystickButton armShortPivotUpButton;
  public JoystickButton armShortPivotDownButton;
  //public JoystickButton intakeStopButton;
  //public JoystickButton slidingClimbButton;
  //public JoystickButton verticalClimbDownButton;
  //public JoystickButton verticalClimbUpButton;
  //public JoystickButton beltcroShooterButton;
  //public JoystickButton manualSlidingClimbButton;
  //public JoystickButton slidingClimbReverseButton;
  //public JoystickButton slidingTraversalDownButton;
  //public JoystickButton climbLockButton;
  //public JoystickButton beltcroReverseButton;

  //public JoystickButton slidingShortClimbButton;
  //public JoystickButton slidingShortClimbReverseButton;
 

  public JoystickButton limelightButton;
  //public JoystickButton climbLockLeftButton;
  //public JoystickButton climbLockRightButton;
  //public JoystickButton climbUnlockLeftButton;
  //public JoystickButton climbUnlockRightButton;
  //public JoystickButton intakeOverrideButton;
  //public JoystickButton ballShooterButton;
  //public JoystickButton highGoalShooterButton;
  //public JoystickButton turretButton;

  public OI(){

    //Driver Stick
  turboButton = new JoystickButton(driverStick, RobotMap.turboButtonNumber);
  driveStraightButton = new JoystickButton(driverStick, RobotMap.driveStraightButtonNumber);
  turnLeft90Button = new JoystickButton(driverStick, RobotMap.turnLeft90ButtonNumber);
  turnRight90Button = new JoystickButton(driverStick, RobotMap.turnRight90ButtonNumber);
  turnLeft180Button = new JoystickButton(driverStick, RobotMap.turnLeft180ButtonNumber);
  turnRight180Button = new JoystickButton(driverStick, RobotMap.turnRight180ButtonNumber);
  limelightButton = new JoystickButton(driverStick, RobotMap.limelightButtonNumber);
  balanceButton = new JoystickButton(driverStick, RobotMap.balanceButtonNumber);
  initialPitchButton = new JoystickButton(driverStick, RobotMap.initialPitchButtonNumber);
  photonvisionButton = new JoystickButton(driverStick, RobotMap.photonvisionButtonNumber);



  //Operator Stick
  turretRightPOV = new POVButton(operatorStick, 90);
  turretLeftPOV = new POVButton(operatorStick, 270);
  //pneumaticGrabButton = new JoystickButton(operatorStick, RobotMap.pneumaticGrabButtonNumber);
  armFullOutButton = new JoystickButton(operatorStick, RobotMap.armFullOutButtonNumber);
  armFullBackButton = new JoystickButton(operatorStick, RobotMap.armFullBackButtonNumber);
  wristUpPOV = new POVButton(operatorStick, 0);
  wristDownPOV = new POVButton(operatorStick, 180);
  wristDownUpButton = new JoystickButton(operatorStick, RobotMap.wristDownUpButtonNumber);
  //wristDeployButton = new JoystickButton(operatorStick, RobotMap.wristDeployButtonNumber);
  armPivotButton = new JoystickButton(operatorStick, RobotMap.armPivotButtonNumber);
  armShortPivotUpButton = new JoystickButton(operatorStick, RobotMap.armShortPivotUpButtonNumber);
  armShortPivotDownButton = new JoystickButton(operatorStick, RobotMap.armShortPivotDownButtonNumber);
  //wristLockButton = new JoystickButton(operatorStick, RobotMap.wristLockButtonNumber);
  //armLockButton = new JoystickButton(operatorStick, RobotMap.armLockButtonNumber);




  //intakeStopButton = new JoystickButton(operatorStick, RobotMap.intakeStopButtonNumber);
  //manualSlidingClimbButton = new JoystickButton(driverStick, RobotMap.manualSlidingClimbButtonNumber);
  //verticalClimbUpButton = new JoystickButton(operatorStick, RobotMap.verticalClimbUpButtonNumber);
  //verticalClimbDownButton = new JoystickButton(operatorStick, RobotMap.verticalClimbDownButtonNumber); //change to operatorStick when we have both joystick
  //limeLightButton = new JoystickButton(driverStick, RobotMap.limeLightButtonNumber);


  //slidingShortClimbButton = new JoystickButton(operatorStick, RobotMap.slidingShortClimbButtonNumber);
  //slidingShortClimbReverseButton = new JoystickButton(operatorStick, RobotMap.slidingShortClimbReverseButtonNumber);

  //climbLockRightButton = new JoystickButton(operatorStick, RobotMap.climbLockRightButtonNumber);
  //climbLockLeftButton = new JoystickButton(operatorStick, RobotMap.climbLockRightButtonNumber);
  //climbUnlockLeftButton = new JoystickButton(operatorStick, RobotMap.climbUnlockLeftButtonNumber);
  //climbUnlockRightButton = new JoystickButton(operatorStick, RobotMap.climbUnlockRightButtonNumber);
  //turretButton = new JoystickButton(operatorStick, RobotMap.turretButtonNumber);

  turnLeft90Button.onTrue(new Turn90DegreesCommand());
  turnRight90Button.onTrue(new Turn90DegreesCommand());
  turnLeft180Button.onTrue(new Turn90DegreesCommand());
  turnRight180Button.onTrue(new Turn90DegreesCommand());
  
  balanceButton.onTrue(new BalanceCommand());

  //pneumaticGrabButton.onTrue(new PneumaticsGrabbingCommand());

  turretLeftPOV.whileTrue(new ManualTurretTurningCommand());
  turretRightPOV.whileTrue(new ManualTurretTurningCommand());

  armFullBackButton.onTrue(new ArmJoustCommand());
  armFullOutButton.onTrue(new ArmJoustCommand());

  wristUpPOV.whileTrue(new ManualWristCommand());
  wristDownPOV.whileTrue(new ManualWristCommand());

  //wristDeployButton.onTrue(new WristMotionMagic());
  wristDownUpButton.onTrue(new WristMotionMagic());

  armPivotButton.onTrue(new ArmSetPositionsCommand());
  armShortPivotDownButton.onTrue(new ArmSetPositionsCommand());
  armShortPivotUpButton.onTrue(new ArmSetPositionsCommand());
  
  }
  
}
