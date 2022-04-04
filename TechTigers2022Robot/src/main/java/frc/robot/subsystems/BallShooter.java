// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;


public class BallShooter extends SubsystemBase {
  /** Creates a new BallShooter. */
    private TalonFX ballShooterTopTalon = new TalonFX (RobotMap.ballShooterTopID);
    private TalonFX ballShooterBottomTalon = new TalonFX (RobotMap.ballShooterBottomID);
  public BallShooter() {
    
    ballShooterTopTalon.configFactoryDefault();
    ballShooterBottomTalon.configFactoryDefault();
    
    //ballShooterBottomTalon.follow(ballShooterTalon);
    //ballShooterBottomTalon.setInverted(InvertType.InvertMotorOutput);
   
    ballShooterTopTalon.setSensorPhase(true);
    ballShooterBottomTalon.setSensorPhase(true);

    
    ballShooterTopTalon.setNeutralMode(NeutralMode.Coast);
    ballShooterBottomTalon.setNeutralMode(NeutralMode.Coast);
    ballShooterTopTalon.configNeutralDeadband(0.00, RobotMap.pidLoopTimeout); //during testing was 0.001
    ballShooterBottomTalon.configNeutralDeadband(0.00, RobotMap.pidLoopTimeout); //during testing was 0.001
    
    ballShooterTopTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);
    ballShooterBottomTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);


    /* Set relevant frame periods to be at least as fast as periodic rate */
		ballShooterTopTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.pidLoopTimeout);
    ballShooterBottomTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.pidLoopTimeout);
		//ballShooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.pidLoopTimeout);

		/* Set the peak and nominal outputs */
		ballShooterTopTalon.configNominalOutputForward(0, RobotMap.pidLoopTimeout);
		ballShooterTopTalon.configNominalOutputReverse(0, RobotMap.pidLoopTimeout);
		ballShooterTopTalon.configPeakOutputForward(1, RobotMap.pidLoopTimeout);
		ballShooterTopTalon.configPeakOutputReverse(-1, RobotMap.pidLoopTimeout);

    ballShooterBottomTalon.configNominalOutputForward(0, RobotMap.pidLoopTimeout);
		ballShooterBottomTalon.configNominalOutputReverse(0, RobotMap.pidLoopTimeout);
		ballShooterBottomTalon.configPeakOutputForward(1, RobotMap.pidLoopTimeout);
		ballShooterBottomTalon.configPeakOutputReverse(-1, RobotMap.pidLoopTimeout);

		/* Set gains in slot0 - see documentation */
    ballShooterTopTalon.selectProfileSlot(RobotMap.kShooterSlotIDx, RobotMap.kPIDLoopIDx);
		ballShooterTopTalon.config_kF(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kF, RobotMap.pidLoopTimeout);
		ballShooterTopTalon.config_kP(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kP, RobotMap.pidLoopTimeout);
		ballShooterTopTalon.config_kI(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kI, RobotMap.pidLoopTimeout);
		ballShooterTopTalon.config_kD(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kD, RobotMap.pidLoopTimeout);

    ballShooterBottomTalon.selectProfileSlot(RobotMap.kShooterSlotIDx, RobotMap.kPIDLoopIDx);
		ballShooterBottomTalon.config_kF(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kF, RobotMap.pidLoopTimeout);
		ballShooterBottomTalon.config_kP(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kP, RobotMap.pidLoopTimeout);
		ballShooterBottomTalon.config_kI(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kI, RobotMap.pidLoopTimeout);
		ballShooterBottomTalon.config_kD(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kD, RobotMap.pidLoopTimeout);

		/* Set acceleration and vcruise velocity - see documentation */
		//ballShooterTalon.configMotionCruiseVelocity(1000, RobotMap.pidLoopTimeout);
		//ballShooterTalon.configMotionAcceleration(1000, RobotMap.pidLoopTimeout);

		/* Zero the sensor once on robot boot up */
		ballShooterTopTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
    ballShooterBottomTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);

    
   
    /*ballShooterTalon.config_kF(RobotMap.kShooterSlotIDx,RobotMap.shooterGainsVelocity.kF);
    ballShooterTalon.config_kP(RobotMap.kShooterSlotIDx,RobotMap.shooterGainsVelocity.kP);
    ballShooterTalon.config_kI(RobotMap.kShooterSlotIDx,RobotMap.shooterGainsVelocity.kI);
    ballShooterTalon.config_kD(RobotMap.kShooterSlotIDx,RobotMap.shooterGainsVelocity.kD);
*/
    
    
    zeroSensors();  
  }

  void zeroSensors() {
    //ballShooterTalon.getSensorCollection().setQuadraturePosition(0, RobotMap.pidLoopTimeout);
    ballShooterTopTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
    ballShooterBottomTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
    System.out.println("Ball Shooter Sensor is zeroed\n");
  }

  public void shoot(boolean mrWuBoolean){
    if (mrWuBoolean)
    {     
      ballShooterTopTalon.set(ControlMode.Velocity, RobotMap.shooterTopSpeed_nativeUnit);
      ballShooterBottomTalon.set(ControlMode.Velocity, RobotMap.shooterBottomSpeed_nativeUnit);
    }
    else
    {
      ballShooterTopTalon.set(ControlMode.Velocity, 0);
      ballShooterBottomTalon.set(ControlMode.Velocity, 0);
    }
    double speedTop = ballShooterTopTalon.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);
    double speedBottom = ballShooterBottomTalon.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);

    SmartDashboard.putNumber("SensorShooterVel", speedTop);
    SmartDashboard.putNumber("SensorShooterVel", speedBottom);

    //System.out.println(""+ speed);
  }
  public void shootHighGoal(boolean mrWuBoolean){
    if (mrWuBoolean)
    {     
      ballShooterTopTalon.set(ControlMode.Velocity, RobotMap.shooterTopHighGoalSpeed_nativeUnit);
      ballShooterBottomTalon.set(ControlMode.Velocity, RobotMap.shooterBottomHighGoalSpeed_nativeUnit);
    }
    else
    {
      ballShooterTopTalon.set(ControlMode.Velocity, 0);
      ballShooterBottomTalon.set(ControlMode.Velocity, 0);
    }
    double speedTopHighGoal = ballShooterTopTalon.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);
    double speedBottomHighGoal = ballShooterBottomTalon.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);

    SmartDashboard.putNumber("SensorShooterVel", speedTopHighGoal);
    SmartDashboard.putNumber("SensorShooterVel", speedBottomHighGoal);

    //System.out.println(""+ speed);
  }

  public boolean targetSpeed(){
    double speedTop = ballShooterTopTalon.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);
    SmartDashboard.putNumber("Shooter Speed Top", speedTop);
    double speedTopDifferential = speedTop - RobotMap.shooterTopSpeed_nativeUnit;
    SmartDashboard.putNumber("Shooter Speed Top Differential", speedTopDifferential);

    double speedBottom = ballShooterBottomTalon.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);
    SmartDashboard.putNumber("Shooter Speed 2", speedBottom);
    double speedBottomDifferential = speedBottom - RobotMap.shooterBottomSpeed_nativeUnit;
    SmartDashboard.putNumber("Shooter Speed Differential", speedBottomDifferential);

    if ((Math.abs(speedBottomDifferential) < RobotMap.shooterBottomSpeedTolerance) && (Math.abs(speedTopDifferential) < RobotMap.shooterTopSpeedTolerance)){
      return true;
    }
    else{
      return false;
    }
    }

    public boolean targetHighGoalSpeed(){
      double speedTopHighGoal = ballShooterTopTalon.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);
      SmartDashboard.putNumber("Shooter Speed Top High Goal", speedTopHighGoal);
      double speedTopHighGoalDifferential = speedTopHighGoal - RobotMap.shooterTopHighGoalSpeed_nativeUnit;
      SmartDashboard.putNumber("Shooter Speed Top High Goal Differential", speedTopHighGoalDifferential);
  
      double speedBottomHighGoal = ballShooterBottomTalon.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);
      SmartDashboard.putNumber("Shooter High Goal Speed 2", speedBottomHighGoal);
      double speedBottomHighGoalDifferential = speedBottomHighGoal - RobotMap.shooterBottomHighGoalSpeed_nativeUnit;
      SmartDashboard.putNumber("Shooter High Goal Speed Differential", speedBottomHighGoalDifferential);
      
      if ((Math.abs(speedBottomHighGoalDifferential) < RobotMap.shooterBottomSpeedTolerance) && (Math.abs(speedTopHighGoalDifferential) < RobotMap.shooterTopSpeedTolerance)){
        return true;
      }
    else {
      //come back to this. this is for making a timeout
    }
    return false;
    
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
