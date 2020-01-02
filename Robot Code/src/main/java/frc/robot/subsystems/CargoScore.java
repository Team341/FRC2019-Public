/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.cargo.CargoScoreSetToZero;

/**
 * Add your docs here.
 */
public class CargoScore extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static CargoScore instance = null;

  public static CargoScore getInstance() {
    if(instance == null) {
      instance = new CargoScore();
    }
    return instance;
  }

  private WPI_VictorSPX mScoringMotor = new WPI_VictorSPX(RobotMap.CargoScore.SCORING_MOTOR_PORT);
  private CANSparkMax mIntakeMotor = new CANSparkMax(RobotMap.CargoIntake.INTAKE_MOTOR_PORT, MotorType.kBrushless);
  private WPI_VictorSPX mConveyorMotor = new WPI_VictorSPX(RobotMap.CargoIntake.CONVEYOR_MOTOR_PORT);
  private DigitalInput mBallSensor = new DigitalInput(RobotMap.CargoScore.BALL_SENSOR_PORT);

  private CargoScore() {
    mScoringMotor.setNeutralMode(NeutralMode.Brake);

    

    mIntakeMotor.setSmartCurrentLimit(-1);
    //mIntakeMotor.setParameter(ConfigParameter.kSoftLimitFwdEn, false);
    //mIntakeMotor.setParameter(ConfigParameter.kSoftLimitRevEn, false);
   
  }

  public boolean ballAcquired() {
    return !mBallSensor.get();
  }

  public void setSpeed(double speed) {
    mScoringMotor.set(ControlMode.PercentOutput, speed);
  }

  public void intakeBall() {
    double speed = RobotMap.CargoScore.IN_SPEED;
    if(ballAcquired()) {
      speed = 0.0;
    }
    setSpeed(speed);
  }

  public void intakeBall(double speed) {
    if(ballAcquired()) {
      speed = 0.0;
    }
    setSpeed(speed);
  }

  public void scoreBall() {
    setSpeed(RobotMap.CargoScore.SCORE_SPEED);
  }

  public void scoreBall(double speed) {
    setSpeed(speed);
  }

  public void unjamBall() {
    setSpeed(RobotMap.CargoScore.UNJAM_SPEED);
  }

  public void runIntake() {
    mIntakeMotor.set(RobotMap.CargoIntake.INTAKING_SPEED);

    //mIntakeMotor.set(ControlMode.PercentOutput, RobotMap.CargoIntake.INTAKING_SPEED);
  }

  public void reverseIntake() {
    mIntakeMotor.set(RobotMap.CargoIntake.SPITTING_SPEED);
    
    //mIntakeMotor.set(ControlMode.PercentOutput, RobotMap.CargoIntake.SPITTING_SPEED);
  }

  public void setIntakeSpeed(double speed) {
    mIntakeMotor.set(speed);
    
    //mIntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runConveyor() {
    mConveyorMotor.set(ControlMode.PercentOutput, RobotMap.CargoIntake.CONVEYOR_IN_SPEED);
  }

  public void reverseConveyor() {
    mConveyorMotor.set(ControlMode.PercentOutput, RobotMap.CargoIntake.CONVEYOR_OUT_SPEED);
  }

  public void setConveyorSpeed(double speed) {
    mConveyorMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new CargoScoreSetToZero());
  }

  public void logToDashboard() {
    SmartDashboard.putBoolean("CargoScore/hasBall", ballAcquired());
    SmartDashboard.putNumber("CargoScore/MotorSpeed", mScoringMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("CargoScore/IntakeCurrent", mIntakeMotor.getOutputCurrent()); 
  }
}
