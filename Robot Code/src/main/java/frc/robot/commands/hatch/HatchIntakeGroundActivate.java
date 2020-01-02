/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchIntake;
import frc.robot.subsystems.HatchScore;

public class HatchIntakeGroundActivate extends Command {

  private HatchIntake mHatchIntake = HatchIntake.getInstance();
  private HatchScore mHatchScore = HatchScore.getInstance();

  private int count;

  public HatchIntakeGroundActivate() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    count = -1;

    requires(mHatchIntake);
    requires(Elevator.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Elevator.getInstance().setPosition(RobotMap.Elevator.ELEVATOR_HATCH_SCOOP_LOAD);
    count = 0;
    if(!mHatchScore.isSliderRetracted()) {
      mHatchScore.retractSlider();
    }

    if (!mHatchIntake.hasHatch()){
      mHatchIntake.runIntake();
      mHatchIntake.lower();
      if(mHatchScore.getExpanderPistonState()) {
        mHatchScore.retractPiston();
      }
    } else {
      if(!mHatchScore.getExpanderPistonState()) {
        mHatchScore.activatePiston();
      }
      count = 7;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mHatchIntake.runIntake();

    if(mHatchIntake.hasHatch()) {
      count++;
    } else {
      count = 0;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return count > 3; //mHatchIntake.hasHatch();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mHatchIntake.setIntakeSpeed(0.0);
    mHatchIntake.raise();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    if(!HatchScore.getInstance().getExpanderPistonState()) {
      HatchScore.getInstance().activatePiston();
    }
    //end();
    mHatchIntake.setIntakeSpeed(0.0);
    mHatchIntake.raise();
    //mHatchScore.activatePiston();
  }
}
