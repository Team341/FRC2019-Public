/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoScore;

public class CargoScoreSetToZero extends Command {

  private CargoScore mCargoScore;

  public CargoScoreSetToZero() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    mCargoScore = CargoScore.getInstance();
    requires(mCargoScore);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mCargoScore.setSpeed(0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mCargoScore.setSpeed(0.0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mCargoScore.setSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mCargoScore.setSpeed(0.0);
  }
}
