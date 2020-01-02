/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.HatchScore;

public class HatchScoreSliderExtend extends Command {

  HatchScore mHatchScore;

  public HatchScoreSliderExtend() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    mHatchScore = HatchScore.getInstance();

    //requires(mHatchScore);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(!mHatchScore.isSliderExtended()) {
      mHatchScore.extendSlider();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(!mHatchScore.isSliderExtended()) {
      mHatchScore.extendSlider();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true; //mHatchScore.isSliderExtended();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //System.out.println("ending slider extend");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
