/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.ElevatorManualControl;
import frc.robot.utilities.DaisyMath;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Elevator instance = null;

  public static Elevator getInstance() {
    if(instance == null) {
      instance = new Elevator();
    }
    return instance;
  }


  private TalonSRX mElevatorMotorMaster = new TalonSRX(RobotMap.Elevator.ELEVATOR_MOTOR_PORT1);
  private TalonSRX mElevatorMotorSlave = new TalonSRX(RobotMap.Elevator.ELEVATOR_MOTOR_PORT2);
  private DigitalInput mElevatorUpperLimitSwitch = new DigitalInput(RobotMap.Elevator.ELEVATOR_UPPER_LIMIT_SWITCH_PORT);
  private DigitalInput mElevatorLowerLimitSwitch = new DigitalInput(RobotMap.Elevator.ELEVATOR_LOWER_LIMIT_SWITCH_PORT);

  private double kP = 0.25; //.375 //0.5;
  private double kI = 0.0; //0 //0;
  private double kD = 0.075; //0.75 //1.0;

  private double kPdown = 0.05; //.0565 //0.005;
  private double kIdown = 0.0; //0 //0;
  private double kDdown = 0.0; // 0 //0.0;

  private int voltCount = -1;

  public Elevator() {
    mElevatorMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    mElevatorMotorMaster.setInverted(true);
    mElevatorMotorMaster.setSensorPhase(true);

    mElevatorMotorSlave.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    mElevatorMotorSlave.setInverted(true);

    configurePIDs();

    mElevatorMotorMaster.setNeutralMode(NeutralMode.Brake);
    mElevatorMotorSlave.setNeutralMode(NeutralMode.Brake);

    mElevatorMotorMaster.configMotionAcceleration(RobotMap.Elevator.MOTION_ACCELERATION_MAX);
    mElevatorMotorSlave.configMotionAcceleration(RobotMap.Elevator.MOTION_ACCELERATION_MAX);

    mElevatorMotorMaster.configOpenloopRamp(0.75);
    mElevatorMotorSlave.configOpenloopRamp(0.75);

    mElevatorMotorMaster.configContinuousCurrentLimit(25);
    mElevatorMotorSlave.configContinuousCurrentLimit(25);

    mElevatorMotorMaster.configPeakOutputForward(1.0); //.75
    mElevatorMotorSlave.configPeakOutputForward(1.0);

    
    mElevatorMotorMaster.configPeakOutputReverse(-0.75); //-.75
    mElevatorMotorSlave.configPeakOutputReverse(-0.75);

    mElevatorMotorMaster.enableCurrentLimit(true);
    mElevatorMotorSlave.enableCurrentLimit(true);

    voltCount = 0;

    mElevatorMotorSlave.follow(mElevatorMotorMaster);

    //mElevatorMotorMaster.configForwardSoftLimitThreshold(RobotMap.Elevator.ELEVATOR_CARGO_POSITION_TOP, 0);
  }

  public void configurePIDs() {
    mElevatorMotorMaster.config_kP(0, kP);
    mElevatorMotorMaster.config_kI(0, kI);
    mElevatorMotorMaster.config_kD(0, kD);

    mElevatorMotorSlave.config_kP(0, kP);
    mElevatorMotorSlave.config_kI(0, kI);
    mElevatorMotorSlave.config_kD(0, kD);

    mElevatorMotorMaster.config_kP(1, kPdown);
    mElevatorMotorMaster.config_kI(1, kIdown);
    mElevatorMotorMaster.config_kD(1, kDdown);

    mElevatorMotorSlave.config_kP(1, kPdown);
    mElevatorMotorSlave.config_kI(1, kIdown);
    mElevatorMotorSlave.config_kD(1, kDdown);
  }

  public void setSpeed(double speed) {
    if (getUpperLimitSwitch() && speed > 0) {
      speed = 0.0;
    } else if (getLowerLimitSwitch() && speed < 0) {
      speed = 0.0;
    }

    mElevatorMotorMaster.set(ControlMode.PercentOutput, speed);
  }

  public void holdPosition() {
    setPosition(getPosition());
  }

  public boolean getUpperLimitSwitch() {
    return !mElevatorUpperLimitSwitch.get();
  }

  public boolean getLowerLimitSwitch() {
    return !mElevatorLowerLimitSwitch.get();
  }

  public void resetPosition() {
    mElevatorMotorMaster.configSetParameter(ParamEnum.eClearPositionOnIdx, 1, 0x00, 0x00, 10);
    mElevatorMotorMaster.configSetParameter(ParamEnum.eClearPositionOnIdx, 0, 0x00, 0x00, 10);
    mElevatorMotorMaster.setSelectedSensorPosition(0, 0, 10);
  }

  public void setPosition(double position) {
    if(position < getPosition()) {
      mElevatorMotorMaster.selectProfileSlot(1, 0);
    } else {
      mElevatorMotorMaster.selectProfileSlot(0, 0);
    }
    mElevatorMotorMaster.set(ControlMode.Position, position);
  }

  public int getPosition() {
    return mElevatorMotorMaster.getSelectedSensorPosition();
  }

  public void setPositionHeight(double height) {
    double ticks = 0.0;
    ticks = (height / RobotMap.Elevator.ELEVATOR_ENCODER_TICKS_TO_HEIGHT_INCHES) * 1.0;

    setPosition(ticks);
  }

  public double getPositionHeight() {
    return (getPosition() * RobotMap.Elevator.ELEVATOR_ENCODER_TICKS_TO_HEIGHT_INCHES) * 1.0;
  }

  public double getVoltage() {
    return mElevatorMotorMaster.getMotorOutputVoltage();
  }
  
  public double getVoltCount() {
    return voltCount;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ElevatorManualControl());
  }

  public void logToDashboard() {

    double tKP = SmartDashboard.getNumber("Elevator/kP", kP);
    double tKI = SmartDashboard.getNumber("Elevator/kI", kI);
    double tKD = SmartDashboard.getNumber("Elevator/kD", kD);

    double tKPdown = SmartDashboard.getNumber("Elevator/kPdown", kPdown);
    double tKIdown = SmartDashboard.getNumber("Elevator/kIdown", kIdown);
    double tKDdown = SmartDashboard.getNumber("Elevator/kDdown", kDdown);

    if(tKP != kP || tKI != kI || tKD != kD) {
      kP = tKP;
      kI = tKI;
      kD = tKD;
      configurePIDs();
    }
    
    if(tKPdown != kPdown || tKIdown != kIdown || tKDdown != kDdown) {
      kPdown = tKPdown;
      kIdown = tKIdown;
      kDdown = tKDdown;
      configurePIDs();
    }

    if(getVoltage() > RobotMap.Elevator.VOLTAGE_LIMIT) {
      voltCount++;
    } else {
      voltCount = 0;
    }

    SmartDashboard.putNumber("Elevator/EncoderPosition", getPosition());
    SmartDashboard.putNumber("Elevator/EstimatedHeight", getPositionHeight());
    SmartDashboard.putBoolean("Elevator/LowerLimitSwitchEngaged", getLowerLimitSwitch());
    SmartDashboard.putBoolean("Elevator/UpperLimitSwitchEngaged", getUpperLimitSwitch());
    SmartDashboard.putNumber("Elevator/OutputVoltageMaster", getVoltage());
    SmartDashboard.putNumber("Elevator/OutputVoltageSlave", mElevatorMotorSlave.getMotorOutputVoltage());
    SmartDashboard.putNumber("Elevator/OutputCurrentMaster", mElevatorMotorMaster.getOutputCurrent());
    SmartDashboard.putNumber("Elevator/OutputCurrentSlave", mElevatorMotorSlave.getOutputCurrent());

    SmartDashboard.putNumber("Elevator/kP", kP);
    SmartDashboard.putNumber("Elevator/kI", kI);
    SmartDashboard.putNumber("Elevator/kD", kD);

    SmartDashboard.putNumber("Elevator/kPdown", kPdown);
    SmartDashboard.putNumber("Elevator/kIdown", kIdown);
    SmartDashboard.putNumber("Elevator/kDdown", kDdown);

    if(getLowerLimitSwitch()) {
      mElevatorMotorMaster.setSelectedSensorPosition(0);
      mElevatorMotorSlave.setSelectedSensorPosition(0);
      //setSpeed(0.0);
    }
    /*
    if(getUpperLimitSwitch()) {
      mElevatorMotorMaster.setSelectedSensorPosition(RobotMap.Elevator.ELEVATOR_UPPER_LIMIT_TICKS);
      mElevatorMotorSlave.setSelectedSensorPosition(RobotMap.Elevator.ELEVATOR_UPPER_LIMIT_TICKS);
    }
    */
  }
}