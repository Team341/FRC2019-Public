/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.drive.DriveArcade;
import frc.robot.utilities.AlphaFilter;
import frc.robot.utilities.DaisyMath;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static Drive instance = null;

  public static Drive getInstance() {
    if(instance == null) {
      instance = new Drive();
    }

    return instance;
  }


  /* Drivebase Variable Declarations */

  private CANSparkMax mLeftDriveMotor1 = new CANSparkMax(RobotMap.Drive.LEFT_DRIVE_MOTOR_PORT_A, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax mLeftDriveMotor2 = new CANSparkMax(RobotMap.Drive.LEFT_DRIVE_MOTOR_PORT_B, CANSparkMax.MotorType.kBrushless);
  //private CANSparkMax mLeftDriveMotor3 = new CANSparkMax(RobotMap.Drive.LEFT_DRIVE_MOTOR_PORT_C, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax mRightDriveMotor1 = new CANSparkMax(RobotMap.Drive.RIGHT_DRIVE_MOTOR_PORT_A, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax mRightDriveMotor2 = new CANSparkMax(RobotMap.Drive.RIGHT_DRIVE_MOTOR_PORT_B, CANSparkMax.MotorType.kBrushless);
  //private CANSparkMax mRightDriveMotor3 = new CANSparkMax(RobotMap.Drive.RIGHT_DRIVE_MOTOR_PORT_C, CANSparkMax.MotorType.kBrushless);

  private AHRS mGyro = new AHRS(SPI.Port.kMXP);

  private double last_world_linear_accel_x, last_world_linear_accel_y, last_world_linear_accel_z;
  private boolean turnDampening;

  // private Encoder mLeftDriveEncoder;
  // private Encoder mRightDriveEncoder;

  private AlphaFilter mAlphaFilter;

  private double kP = .1;

  public Drive() {
    mAlphaFilter = new AlphaFilter(RobotMap.Drive.DRIVE_ALPHA_FILTER_GAIN_LOW_SPEED_FORWARD,
                                    RobotMap.Drive.DRIVE_ALPHA_FILTER_GAIN_LOW_SPEED_BACKWARD);

    turnDampening = false;

    //mLeftDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    //mLeftDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mLeftDriveMotor1.setInverted(true);
    mLeftDriveMotor2.setInverted(true);
    //mLeftDriveMotor3.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mLeftDriveMotor2.follow(mLeftDriveMotor1);
    //mLeftDriveMotor3.follow(mLeftDriveMotor1);
    
    mLeftDriveMotor1.setSmartCurrentLimit(RobotMap.Drive.MOTOR_CURRENT_LIMIT);
    mLeftDriveMotor2.setSmartCurrentLimit(RobotMap.Drive.MOTOR_CURRENT_LIMIT);
    //mLeftDriveMotor3.setSmartCurrentLimit(RobotMap.Drive.MOTOR_CURRENT_LIMIT);
    mLeftDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mLeftDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mLeftDriveMotor1.getEncoder().setPositionConversionFactor(RobotMap.Drive.NEO_ENCODER_INCHES_PER_TICK);
    mLeftDriveMotor1.getEncoder().setVelocityConversionFactor(1);
    mLeftDriveMotor1.getPIDController().setP(kP);
    mLeftDriveMotor2.getPIDController().setP(kP);

    mRightDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mRightDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mRightDriveMotor1.getEncoder().setPositionConversionFactor(RobotMap.Drive.NEO_ENCODER_INCHES_PER_TICK);
    mRightDriveMotor1.getEncoder().setVelocityConversionFactor(1);
    //mRightDriveMotor3.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightDriveMotor1.setInverted(false);
    mRightDriveMotor2.setInverted(false);
    //mRightDriveMotor3.setInverted(true);
    mRightDriveMotor2.follow(mRightDriveMotor1);
    //mRightDriveMotor3.follow(mRightDriveMotor1);
    mRightDriveMotor1.getPIDController().setP(kP);
    mRightDriveMotor2.getPIDController().setP(kP);

    mRightDriveMotor1.setSmartCurrentLimit(RobotMap.Drive.MOTOR_CURRENT_LIMIT);
    mRightDriveMotor2.setSmartCurrentLimit(RobotMap.Drive.MOTOR_CURRENT_LIMIT);
    //mRightDriveMotor3.setSmartCurrentLimit(RobotMap.Drive.MOTOR_CURRENT_LIMIT);


    //mLeftDriveEncoder = new Encoder(RobotMap.Drive.LEFT_DRIVE_ENCODER_PORT_A, RobotMap.Drive.LEFT_DRIVE_ENCODER_PORT_B);
    //mRightDriveEncoder = new Encoder(RobotMap.Drive.RIGHT_DRIVE_ENCODER_PORT_A, RobotMap.Drive.RIGHT_DRIVE_ENCODER_PORT_B);

    //mLeftDriveEncoder.setDistancePerPulse(RobotMap.Drive.ENCODER_DISTANCE_PER_PULSE);
    //mRightDriveEncoder.setDistancePerPulse(RobotMap.Drive.ENCODER_DISTANCE_PER_PULSE);

    mGyro.zeroYaw();
    mGyro.enableLogging(false);
    resetEncoderPositions();
  }

  public void setSpeed( double leftSpeed, double rightSpeed) {
    mLeftDriveMotor1.set(leftSpeed);
    mLeftDriveMotor2.set(leftSpeed);
    //mLeftDriveMotor3.set(leftSpeed);
    mRightDriveMotor1.set(rightSpeed);
    mRightDriveMotor2.set(rightSpeed);
    //mRightDriveMotor3.set(rightSpeed);
  }

  public void setSpeedTurn(double speed, double turn) {
    speed = mAlphaFilter.calculate(speed);

    /*
    if(Elevator.getInstance().getPosition() > RobotMap.Elevator.ELEVATOR_CARGO_POSITION_BOTTOM) {
      speed *= 0.5; 
    }
    */
    
    double speedScalar = (1.0 * Elevator.getInstance().getPosition() - 1.0 * RobotMap.Elevator.ELEVATOR_CARGO_POSITION_BOTTOM) /
                         (1.0 * RobotMap.Elevator.ELEVATOR_CARGO_POSITION_TOP - 1.0 * RobotMap.Elevator.ELEVATOR_CARGO_POSITION_BOTTOM);

    SmartDashboard.putNumber("Drive/SpeedScalar", speedScalar);

    speedScalar =  1 - DaisyMath.minmax(speedScalar, 0.0, 0.85);

    

    

    if(turnDampening){
      
      turn *= RobotMap.Drive.TURN_DAMPEN_CONSTANT;
    
    }
    

    double leftSpeed = DaisyMath.minmax(speed - turn, -1.0, 1.0);
    double rightSpeed = DaisyMath.minmax(speed + turn, -1.0, 1.0);

    setSpeed(leftSpeed, rightSpeed);
  }

  public void setTurnDampening(boolean on) {
    turnDampening = on;

  }

  public boolean getTurnDampening() {
    return turnDampening;
  }

  public void setEncoderPosition(double pos) {
    
    mLeftDriveMotor1.getEncoder().setPosition(pos);
    mLeftDriveMotor2.getEncoder().setPosition(pos);
    mRightDriveMotor1.getEncoder().setPosition(pos);
    mRightDriveMotor2.getEncoder().setPosition(pos);
  }

  /*
  public double getLeftEncoderPosition() {
    return mLeftDriveEncoder.getDistance();
  }

  public double getRightEncoderPosition() {
    return mRightDriveEncoder.getDistance();
  }
  */

  public double getLeftEncoderPosition() {
    return mLeftDriveMotor1.getEncoder().getPosition();
  }

  public double getRightEncoderPosition() {
    return mRightDriveMotor1.getEncoder().getPosition();
  }

  public double getLeftEncoderRawPosition() {
    return mLeftDriveMotor1.getEncoder().getPosition() / RobotMap.Drive.NEO_ENCODER_INCHES_PER_TICK;
  }

  public double getRightEncoderRawPosition() {
    return mRightDriveMotor1.getEncoder().getPosition() / RobotMap.Drive.NEO_ENCODER_INCHES_PER_TICK;
  }

  public double getLeftEncoderSpeed() {
    return mLeftDriveMotor1.getEncoder().getVelocity();
  }

  public double getRightEncoderSpeed() {
    return mRightDriveMotor1.getEncoder().getVelocity();
  }

  public double getGyroYaw() {
    return mGyro.getYaw();
  }

  public double getGyroPitch() {
    return mGyro.getPitch();
  }

  public double getGyroRoll() {
    return mGyro.getRoll();
  }

  public boolean getGyroImpact() {
    boolean collisionDetected = false;

    double curr_world_linear_accel_x = mGyro.getWorldLinearAccelX();

    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;

    double curr_world_linear_accel_y = mGyro.getWorldLinearAccelY();

    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    last_world_linear_accel_y = curr_world_linear_accel_y;

    double curr_world_linear_accel_z = mGyro.getWorldLinearAccelZ();

    double currentJerkZ = curr_world_linear_accel_z - last_world_linear_accel_z;
    last_world_linear_accel_z = curr_world_linear_accel_z;

    if ((Math.abs(currentJerkZ) > RobotMap.Drive.K_COLLISION_THRESHOLD_DELTA_G)) {
        collisionDetected = true;
    }
    SmartDashboard.putBoolean("Drive/CollisionDetected", collisionDetected);
    SmartDashboard.putNumber("Drive/Gyro/JerkX", currentJerkX);
    SmartDashboard.putNumber("Drive/Gyro/JerkY", currentJerkY);
    SmartDashboard.putNumber("Drive/Gyro/JerkZ", currentJerkZ);

    return collisionDetected;
  }

  public void resetEncoderPositions() {
    //mLeftDriveEncoder.reset();
    //mRightDriveEncoder.reset();

    mLeftDriveMotor1.getEncoder().setPosition(0);
    mRightDriveMotor1.getEncoder().setPosition(0);
  }

  public void resetGyroPosition() {
    mGyro.reset();
  }

  public double getLeftMotorVoltage() {
    return mLeftDriveMotor1.getAppliedOutput();
  }

  public double getRightMotorVoltage() {
    return mRightDriveMotor1.getAppliedOutput();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveArcade());
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("Drive/LeftEncoderPosition", getLeftEncoderPosition());
    SmartDashboard.putNumber("Drive/RightEncoderPosition", getRightEncoderPosition());
    /*
    SmartDashboard.putNumber("Drive/LeftNeoEncoderPosition", getNeoLeftEncoderPosition());
    SmartDashboard.putNumber("Drive/RightNeoEncoderPosition", getNeoRightEncoderPosition());
    */
    SmartDashboard.putNumber("Drive/LeftEncoderSpeed", getLeftEncoderSpeed());
    SmartDashboard.putNumber("Drive/RightEncoderSpeed", getRightEncoderSpeed());
    SmartDashboard.putNumber("Drive/Robot Heading", getGyroYaw());
    SmartDashboard.putNumber("Drive/Robot Pitch", getGyroPitch());
    SmartDashboard.putNumber("Drive/Robot Roll", getGyroRoll());
    SmartDashboard.putNumber("Drive/currentLeft", mLeftDriveMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Drive/currentRight", mRightDriveMotor1.getOutputCurrent());
    SmartDashboard.putString("Drive/CurrentCommand", getCurrentCommandName());
  }
}