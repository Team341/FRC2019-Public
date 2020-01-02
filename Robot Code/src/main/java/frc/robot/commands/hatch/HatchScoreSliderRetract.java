/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.HatchScore;

public class HatchScoreSliderRetract extends Command {

  HatchScore mHatchScore;

  public HatchScoreSliderRetract() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    mHatchScore = HatchScore.getInstance();

    //requires(mHatchScore);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(!mHatchScore.isSliderRetracted()) {
      mHatchScore.retractSlider();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(!mHatchScore.isSliderRetracted()) {
      mHatchScore.retractSlider();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //System.out.println("ending slider retract");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mHatchScore.extendSlider();
  }
}
