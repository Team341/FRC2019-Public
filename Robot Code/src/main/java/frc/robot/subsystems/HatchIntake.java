/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HatchIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static HatchIntake instance = null;

  public static HatchIntake getInstance() {
    if(instance == null) {
      instance = new HatchIntake();
    }
    return instance;
  }

  private VictorSPX mIntakeMotor = new VictorSPX(RobotMap.HatchIntake.INTAKE_MOTOR_PORT);
  private Solenoid mLowerer = new Solenoid(RobotMap.HatchIntake.LOWERER_PISTON_PORT);
  private DigitalInput mHatchSensor = new DigitalInput(RobotMap.HatchIntake.HATCH_SENSOR_PORT);
  
  private double current;
  //private int hasHatchCount;
  //private int totCount;
  //private double[] currents;
  //private boolean hasHatch;

  private boolean sensorWorking;

  private HatchIntake() {
    current = 0.0;
    //totCount = 0;
    //currents = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //hasHatch = false;
    mLowerer.set(false);
    //hasHatchCount = 0;
    sensorWorking = true;
  }

  public void reverseIntake() {
    setIntakeSpeed(RobotMap.HatchIntake.ROLLER_OUT_SPEED);
  }

  public void runIntake() {
    if(!hasHatch()) {
      setIntakeSpeed(RobotMap.HatchIntake.ROLLER_IN_SPEED);
      current = mIntakeMotor.getMotorOutputVoltage();
    } else {
      setIntakeSpeed(0.0);
    }
  }

  public void setIntakeSpeed(double speed) {
    mIntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean getPistonState() {
    return mLowerer.get();
  }

  public void lower() {
    mLowerer.set(true);
  }

  public void raise() {
    mLowerer.set(false);
  }
  
  /*
  public double getCurrent() {
    return current;
  }
  */

  public boolean hasHatch() {

    /*
    double avg1 = (currents[0] + currents[1] + currents[2]) / 3.0;
    double avg2 = (currents[3] + currents[4] + currents[5]) / 3.0;

    if(avg1 > avg2 * 2.0) {
      hasHatch = true;
    }
    *//*
    if(current > 12.0) {
      hasHatchCount++;
    } else {
      hasHatchCount = 0;
    }

    if(hasHatchCount > 24) {
      hasHatch = true;
    }

    return hasHatch;
    */
    // if(sensorWorking) {
      return !mHatchSensor.get();
    // } else {
      // return false;
    // }
  }

  public void setSensorWorking(boolean isWorking) {
    sensorWorking = isWorking;
  }

  public boolean getSensorWorking() {
    return sensorWorking;
  }

  /*
  public void setHasHatch(boolean hH) {
    hasHatch = hH;
  }

  public void refreshCurrents() {
    currents[totCount % currents.length] = getCurrent();
    totCount++;
  }
  */

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void logToDashboard() {
    //SmartDashboard.putBoolean("HatchIntake/IsLowered", mLowerer.get());
    SmartDashboard.putNumber("HatchIntake/Speed", mIntakeMotor.getMotorOutputPercent());

    //current = Robot.PDP.getCurrent(8);
    //current = 0;
    //refreshCurrents();

    SmartDashboard.putNumber("HatchIntake/Current", current);
    SmartDashboard.putBoolean("HatchIntake/HasHatch", hasHatch());
    //SmartDashboard.putNumberArray("HatchIntake/Past6Currents", currents);
    //SmartDashboard.putNumber("HatchIntake/Speed", mIntakeMotor.getMotorOutputPercent());
    //SmartDashboard.putNumber("HatchIntake/Speed", mIntakeMotor.getMotorOutputPercent());

  }
}