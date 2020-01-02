/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.HatchIntake;

public class HatchIntakeRun extends Command {

  HatchIntake mHatchIntake;

  public HatchIntakeRun() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    mHatchIntake = HatchIntake.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mHatchIntake.runIntake();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mHatchIntake.runIntake();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mHatchIntake.hasHatch(); //mHatchIntake.getMotorCurrent() > RobotMap.HatchIntake.INTAKE_MOTOR_CURRENT_THRESHOLD; //PDP current is over threshold
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
    end();
  }
}
