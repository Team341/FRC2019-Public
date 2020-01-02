/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoIntake;

public class CargoIntakeReset extends Command {

  private CargoIntake mCargoIntake = CargoIntake.getInstance();

  public CargoIntakeReset() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(mCargoIntake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mCargoIntake.setHingeSpeed(-0.2);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mCargoIntake.getHingeUpperLimitSwitch();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mCargoIntake.setHingeSpeed(0.0);
    mCargoIntake.resetEncoderPositions();
    //mCargoIntake.linkMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mCargoIntake.setHingeSpeed(0.0);
  }
}
