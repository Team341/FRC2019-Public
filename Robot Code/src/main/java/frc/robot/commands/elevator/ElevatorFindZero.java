/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorFindZero extends Command {

  private Elevator mElevator = Elevator.getInstance();

  public ElevatorFindZero() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(mElevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mElevator.setSpeed(-0.15);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mElevator.setSpeed(-0.15);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mElevator.getLowerLimitSwitch();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mElevator.setSpeed(0.0);
    mElevator.resetPosition();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mElevator.setSpeed(0.0);
  }
}
