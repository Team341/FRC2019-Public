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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.cargo.CargoIntakeManualControl;
import frc.robot.utilities.DaisyMath;

/**
 * Add your docs here.
 */
public class CargoIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static CargoIntake instance = null;

  public static CargoIntake getInstance() {
    if(instance == null) {
      instance = new CargoIntake();
    }
    return instance;
  }

  //private WPI_TalonSRX mHingeMotorLeft = new WPI_TalonSRX(RobotMap.CargoIntake.HINGE_MOTOR_LEFT_PORT);
  //private WPI_TalonSRX mHingeMotorRight = new WPI_TalonSRX(RobotMap.CargoIntake.HINGE_MOTOR_RIGHT_PORT);
  //private WPI_VictorSPX mIntakeMotor = new WPI_VictorSPX(RobotMap.CargoIntake.INTAKE_MOTOR_PORT);
  private CANSparkMax mHingeMotorLeft = new CANSparkMax(RobotMap.CargoIntake.HINGE_MOTOR_LEFT_PORT, MotorType.kBrushless);
  private CANSparkMax mHingeMotorRight = new CANSparkMax(RobotMap.CargoIntake.HINGE_MOTOR_RIGHT_PORT, MotorType.kBrushless);
  
  private CANPIDController mHingeMotorLeftPIDs = mHingeMotorLeft.getPIDController(), mHingeMotorRightPIDs = mHingeMotorRight.getPIDController();

  private DigitalInput mHingeUpperLimitSwitch = new DigitalInput(RobotMap.CargoIntake.HINGE_UPPER_LIMIT_SWITCH_PORT);
  private DigitalInput mHingeLowerLimitSwitch = new DigitalInput(RobotMap.CargoIntake.HINGE_LOWER_LIMIT_SWITCH_PORT);

  private double mSetpoint = 0.0;
  private double mOffset = 0.0;

  private double kP = 0.1;
  private double kI = 0.0000;
  private double kD = 0.0;

  private CargoIntake() {

    mHingeMotorLeft.setIdleMode(IdleMode.kBrake);
    mHingeMotorLeft.setInverted(true);
    mHingeMotorRight.setIdleMode(IdleMode.kBrake);
    mHingeMotorRight.setInverted(false);

    //mHingeMotorRight.follow(mHingeMotorLeft, true);

    mHingeMotorLeft.setSmartCurrentLimit(RobotMap.CargoIntake.HINGE_CURRENT_LIMIT);
    mHingeMotorRight.setSmartCurrentLimit(RobotMap.CargoIntake.HINGE_CURRENT_LIMIT);

    //mHingeMotorLeft.setParameter(ConfigParameter.kPositionConversionFactor, RobotMap.CargoIntake.ENCODER_CONVERSION_FACTOR);
    //mHingeMotorRight.setParameter(ConfigParameter.kPositionConversionFactor, RobotMap.CargoIntake.ENCODER_CONVERSION_FACTOR);
    configurePIDs();

    mHingeMotorLeftPIDs.setSmartMotionMaxAccel(RobotMap.CargoIntake.HINGE_MAX_ACCELERATION, 0);
    mHingeMotorLeftPIDs.setSmartMotionMaxVelocity(RobotMap.CargoIntake.HINGE_MAX_VELOCITY, 0);

    mHingeMotorRightPIDs.setSmartMotionMaxAccel(RobotMap.CargoIntake.HINGE_MAX_ACCELERATION, 0);
    mHingeMotorRightPIDs.setSmartMotionMaxVelocity(RobotMap.CargoIntake.HINGE_MAX_VELOCITY, 0);
  }

  public void configurePIDs() {

    mHingeMotorLeftPIDs.setP(kP);
    mHingeMotorLeftPIDs.setI(kI);
    mHingeMotorLeftPIDs.setD(kD);

    mHingeMotorRightPIDs.setP(kP);
    mHingeMotorRightPIDs.setI(kI);
    mHingeMotorRightPIDs.setD(kD);
  }

  public void setPosition(double pos) {

    pos += mOffset;

    mHingeMotorLeftPIDs.setReference(pos, ControlType.kPosition);
    mHingeMotorRightPIDs.setReference(pos, ControlType.kPosition);
    
    SmartDashboard.putNumber("CargoIntake/SetPosition/Trying", pos);

    mSetpoint = pos;
  }

  public void setSmartPosition(double pos) {

    pos += mOffset;

    mHingeMotorLeftPIDs.setReference(pos, ControlType.kSmartMotion);
    mHingeMotorRightPIDs.setReference(pos, ControlType.kSmartMotion);
    
    SmartDashboard.putNumber("CargoIntake/SetSmartPosition/Trying", pos);
  }

  public void holdPosition() {
    
    mHingeMotorLeftPIDs.setReference(getLeftPosition(), ControlType.kPosition);
    mHingeMotorRightPIDs.setReference(getLeftPosition(), ControlType.kPosition);
    
    //mHingeMotorLeft.set(ControlMode.Position, getLeftPosition() / Math.sin(getHingeAngleInDeg())); //need a better way
  }

  public double getSetpoint(){
    return mSetpoint;
  }

  public double getLeftPosition() {
    return mHingeMotorLeft.getEncoder().getPosition();  //mHingeMotorLeft.getSelectedSensorPosition();
  }

  public double getRightPosition() {
    return mHingeMotorRight.getEncoder().getPosition();  //mHingeMotorRight.getSelectedSensorPosition();
  }

  public void resetSensorPositions() {

    mHingeMotorLeft.getEncoder().setPosition(0);
    mHingeMotorRight.getEncoder().setPosition(0);
  }

  public void setHingeSpeed(double speed) {
    
    if (getHingeUpperLimitSwitch() && speed < 0) {
      speed = 0.0;
    } else if (getHingeLowerLimitSwitch() && speed > 0) {
      speed = 0.0;
    }
    
    mHingeMotorLeft.set(speed);
    mHingeMotorRight.set(speed);
  }

  public double getHingeAngleInRad() {
    return Math.toRadians(getLeftPosition());
  }

  public double getHingeAngleInDeg() {
    return DaisyMath.boundAngle0to360Degrees(getLeftPosition());
  }

  public boolean getHingeUpperLimitSwitch() {
    return !mHingeUpperLimitSwitch.get();
  }

  public boolean getHingeLowerLimitSwitch() {
    return !mHingeLowerLimitSwitch.get();
  }

  public void setFrontWheelHeight(double height) {
    double pos = Math.acos((height - RobotMap.CargoIntake.HINGE_HEIGHT_FROM_GROUND)
                    / RobotMap.CargoIntake.HINGE_TO_FRONT_WHEEL_LENGTH)
                    / RobotMap.CargoIntake.ENCODER_ROTATIONS_TO_RADIANS;
    setPosition(pos);
  }

  public double getFrontWheelHeight() {
    return (RobotMap.CargoIntake.HINGE_TO_FRONT_WHEEL_LENGTH * Math.cos(getHingeAngleInRad()))
            + RobotMap.CargoIntake.HINGE_HEIGHT_FROM_GROUND;
  }

  public double getFrontWheelExtent() {
    return (RobotMap.CargoIntake.HINGE_TO_FRONT_WHEEL_LENGTH * Math.sin(getHingeAngleInRad()))
            + RobotMap.CargoIntake.HINGE_DISTANCE_FROM_BACK;
  }

  public void resetEncoderPositions() {

    mHingeMotorLeft.getEncoder().setPosition(0);
    mHingeMotorRight.getEncoder().setPosition(0);
  }

  public void linkMotors() {
    mHingeMotorRight.follow(mHingeMotorLeft, true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    //setDefaultCommand(new CargoIntakeFindZero());

    setDefaultCommand(new CargoIntakeManualControl());
  }

  public void logToDashboard() {

    double tKP = SmartDashboard.getNumber("CargoIntake/kP", kP);
    double tKI = SmartDashboard.getNumber("CargoIntake/kI", kI);
    double tKD = SmartDashboard.getNumber("CargoIntake/kD", kD);

    if(tKP != kP || tKI != kI || tKD != kD) {
      kP = tKP;
      kI = tKI;
      kD = tKD;
      configurePIDs();
    }

    SmartDashboard.putNumber("CargoIntake/LeftEncoderPosition", getLeftPosition());
    SmartDashboard.putNumber("CargoIntake/RightEncoderPosition", getRightPosition());
    SmartDashboard.putNumber("CargoIntake/IntakeAngleDegrees", getHingeAngleInDeg());
    SmartDashboard.putNumber("CargoIntake/FrontWheelHeight", getFrontWheelHeight());
    SmartDashboard.putNumber("CargoIntake/FrontWheelExtent", getFrontWheelExtent());
    SmartDashboard.putBoolean("CargoIntake/HingeUpperLimitSwitch", getHingeUpperLimitSwitch());
    SmartDashboard.putBoolean("CargoIntake/HingeLowerLimitSwitch", getHingeLowerLimitSwitch());
    SmartDashboard.putString("CargoIntake/CurrentCommand", getCurrentCommandName());
    SmartDashboard.putNumber("CargoIntake/Voltage", mHingeMotorLeft.getAppliedOutput());

    SmartDashboard.putNumber("CargoIntake/kP", kP);
    SmartDashboard.putNumber("CargoIntake/kI", kI);
    SmartDashboard.putNumber("CargoIntake/kD", kD);

    if(getHingeUpperLimitSwitch()) {// && Math.abs(getLeftPosition()) > 1.5) {
      resetSensorPositions();
    }
  }
}