/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hab extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Hab instance = null;

  public static Hab getInstance() {
    if(instance == null) {
      instance = new Hab();
    }
    return instance;
  }

  private Solenoid mPTO = new Solenoid(RobotMap.Hab.PTO_PISTON_PORT);
  //private Ultrasonic mUltra = new Ultrasonic(RobotMap.Hab.ULTRASONIC_TRIGGER_PORT, RobotMap.Hab.ULTRASONIC_ECHO_PORT);
  //private DistanceSensor mDistanceSensor = new DistanceSensor(RobotMap.Hab.DISTANCE_SENSOR_PORT);

  private Drive mDrive;

  private Hab() {
    mDrive = Drive.getInstance();
    mPTO.set(false);
  }

  public boolean overHabPlatform() {
    return getPTOState() && getRangeToFloorInCM() < RobotMap.Hab.OVER_HAB_RANGE_THRESHOLD; //when sensor knows we are definitely over the hab platform enough.
  }

  public double getRackHeight() {
    return mDrive.getLeftEncoderPosition() * RobotMap.Hab.DRIVE_ROTATIONS_TO_RACK_HEIGHT + RobotMap.Hab.RACK_STARTING_HEIGHT;
  }

  public void setRackHeight(double height) {
    mDrive.setEncoderPosition((height - RobotMap.Hab.RACK_STARTING_HEIGHT) * RobotMap.Hab.DRIVE_ROTATIONS_TO_RACK_HEIGHT);
  }

  public boolean getPTOState() {
    return mPTO.get();
  }

  public void activatePTO() {
    mPTO.set(true);
    mDrive.resetEncoderPositions();
  }

  public void deactivatePTO() {
    mPTO.set(false);
  }

  public void extendRacks() {
    mDrive.setSpeed(RobotMap.Hab.EXTEND_RACKS_DRIVE_SPEED, RobotMap.Hab.EXTEND_RACKS_DRIVE_SPEED);
  }

  public void setRackSpeed(double leftSpeed, double rightSpeed) {
    mDrive.setSpeed(leftSpeed, rightSpeed);
  }

  public void stopRacks() {
    mDrive.setSpeed(0.0, 0.0);
  }

  public void retractRacks() {
    mDrive.setSpeed(RobotMap.Hab.RETRACT_RACKS_DRIVE_SPEED, RobotMap.Hab.RETRACT_RACKS_DRIVE_SPEED);
  }

  public void holdRacks() {
    mDrive.setSpeed(0.0, 0.0); //Come up with a better way to hold them
  }

  public double getGyroRoll() {
    return mDrive.getGyroRoll();
  }

  public double getGyroPitch() {
    return mDrive.getGyroPitch();
  }

  public double getRangeToFloorInCM(){
    return 0.0; //mDistanceSensor.getDistanceInCM();
  }

  public void resetDriveEncoders() {
    mDrive.resetEncoderPositions();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("Hab/MeasuredRangeToFloor", getRangeToFloorInCM());
    SmartDashboard.putNumber("Hab/getRackHeight", getRackHeight());
    //mDistanceSensor.logToDashboard();
    //SmartDashboard.putNumber("Hab/DistanceSensorPort", getOpenPorts());
    SmartDashboard.putBoolean("Hab/OverHabPlatform", overHabPlatform());
    SmartDashboard.putBoolean("Hab/PTOEngaged", getPTOState());
    SmartDashboard.putNumber("Hab/LeftMotorMotorVoltage", mDrive.getLeftMotorVoltage());
    SmartDashboard.putNumber("Hab/RightMotorMotorVoltage", mDrive.getRightMotorVoltage());
  }
}