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
public class LEDs extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static LEDs instance = null;

  public static LEDs getInstance() {
    if(instance == null) {
      instance = new LEDs();
    }
    return instance;
  }

  private Solenoid mRed = new Solenoid(RobotMap.LEDS.RED_PORT);
  private Solenoid mGreen = new Solenoid(RobotMap.LEDS.GREEN_PORT);
  private Solenoid mBlue = new Solenoid(RobotMap.LEDS.BLUE_PORT);
  
  private boolean isRed, isGreen, isBlue;

  private LEDs() {
    mRed.set(false);
    mGreen.set(false);
    mBlue.set(false);

    isRed = false;
    isGreen = false;
    isBlue = false;
    
    SmartDashboard.putBoolean("LEDs/Red", isRed);
    SmartDashboard.putBoolean("LEDs/Green", isGreen);
    SmartDashboard.putBoolean("LEDs/Blue", isBlue);
    
    SmartDashboard.putBoolean("LEDs/isRed", isRed);
    SmartDashboard.putBoolean("LEDs/isGreen", isGreen);
    SmartDashboard.putBoolean("LEDs/isBlue", isBlue);

  }

  public void setColor(int color) {
    switch(color) {
      case RobotMap.LEDS.CARGO_HATCH:
                mRed.set(true);
                mGreen.set(true);
                mBlue.set(false);
                isRed = true;
                isGreen = true;
                isBlue = false;
                break;
      case RobotMap.LEDS.VISION_BALL:
                mRed.set(false);
                mGreen.set(true);
                mBlue.set(false);
                isRed = false;
                isGreen = true;
                isBlue = false;
                break;
      case RobotMap.LEDS.VISION_HATCH:
                mRed.set(false);
                mGreen.set(false);
                mBlue.set(true);
                isRed = false;
                isGreen = false;
                isBlue = true;
                break;
      
      case RobotMap.LEDS.ELEVATOR_VOLTAGE:
                mRed.set(true);
                mGreen.set(false);
                mBlue.set(false);
                isRed = true;
                isGreen = false;
                isBlue = false;
                break;
      case RobotMap.LEDS.NO_COLOR:
                mRed.set(false);
                mGreen.set(false);
                mBlue.set(false);
                isRed = false;
                isGreen = false;
                isBlue = false;
                break;
      default:
                mRed.set(false);
                mGreen.set(false);
                mBlue.set(false);
                isRed = false;
                isGreen = false;
                isBlue = false;
                break;
    }
  }

  public boolean getRed() {
    return mRed.get();
  }
  public void setRed(boolean on) {
    mRed.set(on);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void logToDashboard() {
    /*
    isRed = (SmartDashboard.getBoolean("LEDs/isRed", false));

    isGreen = (SmartDashboard.getBoolean("LEDs/isGreen", false));

    isBlue = (SmartDashboard.getBoolean("LEDs/isBlue", false));

    if(isRed != mRed.get()) {
      mRed.set(isRed);
      SmartDashboard.putBoolean("LEDs/Red", isRed);
    }
    if(isGreen != mGreen.get()) {
      mGreen.set(isGreen);
      SmartDashboard.putBoolean("LEDs/Green", isGreen);
    }
    if(isBlue != mBlue.get()) {
      mBlue.set(isBlue);
      SmartDashboard.putBoolean("LEDs/Blue", isBlue);
    }
    */
  }
}