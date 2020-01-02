/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.HatchScore;

public class HatchIntakeHasHatch extends Command {
  
  private boolean hasHatch;

  private int count;

  public HatchIntakeHasHatch(boolean hasHatch) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.hasHatch = hasHatch;
    count = -1;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //HatchIntake.getInstance().setHasHatch(hasHatch);
    count = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    count++;

    if(count >= 25) {
      HatchScore.getInstance().activatePiston();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return count >= 26;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    HatchScore.getInstance().activatePiston();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
