/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hab;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Hab;

public class HabActivatePTO extends Command {

  private Hab mHab;

  public HabActivatePTO() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    mHab = Hab.getInstance();

    requires(mHab);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(!mHab.getPTOState()) {
      mHab.activatePTO();
    }
}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(!mHab.getPTOState()) {
      mHab.activatePTO();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mHab.getPTOState();
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
