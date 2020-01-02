/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HatchScore extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static HatchScore instance = null;

  public static HatchScore getInstance() {
    if(instance == null) {
      instance = new HatchScore();
    }
    return instance;
  }

  private Solenoid mHatchPiston = new Solenoid(RobotMap.HatchScore.PISTON_PORT);

  private DoubleSolenoid mSlider = new DoubleSolenoid(RobotMap.HatchScore.SLIDER_PORT_A, RobotMap.HatchScore.SLIDER_PORT_B);

  private HatchScore() {
  }

  public boolean getExpanderPistonState() {
    return !mHatchPiston.get();
  }

  public boolean getSliderPistonState() {
    return mSlider.get() == Value.kForward;
  }

  public void switchPistonState() {
    mHatchPiston.set(!getExpanderPistonState());
  }

  public void activatePiston() {
    mHatchPiston.set(false);
  }

  public void retractPiston() {
    mHatchPiston.set(true);
  }

  public boolean isSliderExtended() {
    return mSlider.get().equals(Value.kForward);
  }

  public boolean isSliderRetracted() {
    return mSlider.get().equals(Value.kReverse);
  }

  public void extendSlider() {
    mSlider.set(Value.kForward);
  }

  public void retractSlider() {
    mSlider.set(Value.kReverse);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void logToDashboard() {
    SmartDashboard.putBoolean("HatchScore/PistonExtended", getExpanderPistonState());
    SmartDashboard.putBoolean("HatchScore/SliderExtended", getSliderPistonState());
  }
}
