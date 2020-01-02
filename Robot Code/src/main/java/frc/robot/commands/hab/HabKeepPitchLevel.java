/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hab;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Hab;

public class HabKeepPitchLevel extends Command {

  private Hab mHab;

  private double kPPitch = 0.0, kPRoll = 0.0;

  public HabKeepPitchLevel() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    mHab = Hab.getInstance();

    requires(mHab);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mHab.activatePTO();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double speed = 0.0, leftSpeed = 0.0, rightSpeed = 0.0;

    speed = kPPitch * mHab.getGyroPitch();

    leftSpeed = speed + mHab.getGyroRoll() * kPRoll;
    rightSpeed = speed - mHab.getGyroRoll() * kPRoll;


    mHab.setRackSpeed(leftSpeed, rightSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
