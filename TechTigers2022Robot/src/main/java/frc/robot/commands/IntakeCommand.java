// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
//import java.util.concurrent.atomic.AtomicInteger;
//import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  NetworkTableEntry isRedAlliance;
  public IntakeCommand() {
    addRequirements(RobotContainer.intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

      //get a reference to the subtable called "datatable"
      NetworkTable fmsInfo = inst.getTable("FMSInfo");

      //get a reference to key in "datatable" called "Y"
      isRedAlliance = fmsInfo.getEntry("IsRedAlliance");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int colorNumber;
    boolean allianceColor; //if allianceColor is true, we are RED team
    allianceColor = isRedAlliance.getBoolean(false);
    colorNumber = RobotContainer.intake.getRainbow();
    if (allianceColor){
      if (colorNumber == 1) {
        RobotContainer.intake.intakeWheels(RobotMap.intakeSpeedOut); //if it sees a blue ball and we're on the red team, it reverses intake wheels
      }
      else { // TO DO if we see a red ball on red team, we need to turn on beltcro
        RobotContainer.intake.intakeWheels(RobotMap.intakeSpeedIn); //is not a blue ball and we're on the red team, it keeps intake going the same way
      }
    }
    else
      {if (colorNumber == 2) {
        RobotContainer.intake.intakeWheels(RobotMap.intakeSpeedOut); //if it sees a red ball and we're on the blue team, it spits it away
      }
      else { // TO DO if we see a blue ball on blue team, we need to turn on beltcro
        RobotContainer.intake.intakeWheels(RobotMap.intakeSpeedIn);} // if it isn't a red ball and we're on the blue team, it runs intake
      } 
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.intakeWheels(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
