/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;

public class RumblePulse extends Command {

  private OI mOI = OI.getInstance();

  private double rumble;
  private int count;
  private int lostTime;

  public RumblePulse() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    count = -1;
    lostTime = -1;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    count = 0;
    lostTime = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    rumble = 0.0;

    if(count % 20 < 10) {
      rumble = 1.0;
    }

    mOI.rumbleDriverController(rumble);

    count++;

    if(mOI.getDriveToVisionTarget().hasTarget()) {
      lostTime = count;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return count >= lostTime + 25;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mOI.rumbleDriverController(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
