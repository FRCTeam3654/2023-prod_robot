// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.concurrent.atomic.AtomicInteger;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BallShooterCommand extends CommandBase {
  /** Creates a new BallShooterCommand. */
  private AtomicInteger _mode = new AtomicInteger(0); //mode = 0 means regular teleop; mode = 1 means autonomous mode
  private double startTimeAutonomous = 0;
  private boolean ballShooterAutonomousFlag = false;
  //NetworkTable mercyLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  //NetworkTableEntry MercyLimelightx = mercyLimelightTable.getEntry("tx");
  //NetworkTableEntry MercyLimelighty = mercyLimelightTable.getEntry("ty");
  //NetworkTableEntry MercyLimelightArea = mercyLimelightTable.getEntry("ta");
  //private double readJoeyX = 2;
  //private double readJoeyY = 0;
  //private double readJoeyArea = 0;
  //private double startTimeLimelight = 0;
  public BallShooterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.ballStorage);
    addRequirements(RobotContainer.ballShooter);
    _mode.set(0);
  }
  public BallShooterCommand(int mode) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(RobotContainer.ballShooter);
    //addRequirements(RobotContainer.ballStorage);
    _mode.set(mode);
    //turret=turny thing, shooter=spinny thing, storage=pushy thing -tory<3
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimeAutonomous = Timer.getFPGATimestamp();
    //startTimeLimelight = Timer.getFPGATimestamp();
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //3 is force on
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); //setting the pipeline to 0 for targetting the goal
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTickCount;

    double  turretTickChange;
  if (RobotContainer.oi.ballShooterButton.get() || _mode.get() == 1){
    RobotContainer.ballShooter.shoot(true);
      
    //read values periodically
    //readJoeyX = MercyLimelightx.getDouble(0.0);
    //readJoeyY = MercyLimelighty.getDouble(0.0);
    //readJoeyArea = MercyLimelightArea.getDouble(0.0);

    //post to smart dashboard periodically
    //SmartDashboard.putNumber("LimelightX", readJoeyX);
    //SmartDashboard.putNumber("LimelightY", readJoeyY);
    //SmartDashboard.putNumber("LimelightArea", readJoeyArea);


      //turretTickChange = readJoeyY * RobotMap.turretTickchangemultiplier;
      //currentTickCount = (double)RobotContainer.turret.turretTickCount();
      //figure out positive or negative
      //RobotContainer.turret.turretTurning (currentTickCount + turretTickChange);

//Come back to set ball storage once it is built/programmed
    if (RobotContainer.ballShooter.targetSpeed()){
      //RobotContainer.ballStorage.driveBallStorage1(-1.0);//-1  to move belt forward //1.0
      //RobotContainer.ballStorage.driveBallStorage2(-0.8);//-0.5
    }
    
   
      if (!ballShooterAutonomousFlag){
        ballShooterAutonomousFlag = true;
        _mode.set(1);
      }
  }
    else
    {
      RobotContainer.ballShooter.shoot(false); 
      //RobotContainer.ballStorage.driveBallStorage1(0); 
      //RobotContainer.ballStorage.driveBallStorage2(0);
      //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);  //1 is force off LED - required by FRC
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);  //1 is force off LED - required by FRC
    ballShooterAutonomousFlag = false;
    RobotContainer.ballShooter.shoot(false);  
    //RobotContainer.ballStorage.driveBallStorage1(0);
    //RobotContainer.ballStorage.driveBallStorage2(0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ballShooterAutonomousFlag == true && ( _mode.get() == 1)) {
      if( (startTimeAutonomous + 0.1) > Timer.getFPGATimestamp()) {
          // if autonomous started, let it run at least 100 ms
          return false;
      }
      // enforce timeout in case MP is stucked or running too long
      else if(startTimeAutonomous + RobotMap.autonomousBallShooterTimeOut < Timer.getFPGATimestamp()) {
          ballShooterAutonomousFlag = false;
          //Robot.ballShooter.shoot(false);  
          //RobotContainer.ballStorage.driveBallStorage1(0);
          //RobotContainer.ballStorage.driveBallStorage2(0); 
          _mode.set(0);
           return true;
      }
      
    }
    return false;
  }
}
