/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.CargoIntake;

//Command corresponding to Joystick #1 (Left-Right movement)
public class CargoIntakeSetPosition extends Command {

  private CargoIntake mCargoIntake = CargoIntake.getInstance();
  boolean endCommand = false;

  private int targetPos = 0;

  public CargoIntakeSetPosition(int position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    targetPos = position;
    endCommand = false;
    requires(mCargoIntake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mCargoIntake.setPosition(targetPos);
    endCommand = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(mCargoIntake.getHingeUpperLimitSwitch() && targetPos < mCargoIntake.getLeftPosition()) {
      endCommand = true;
    } else if(mCargoIntake.getHingeLowerLimitSwitch() && targetPos > mCargoIntake.getLeftPosition()) {
      endCommand = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return endCommand || Math.abs(mCargoIntake.getLeftPosition() - targetPos) < RobotMap.CargoIntake.POSITION_MARGIN_OF_ERROR;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mCargoIntake.setHingeSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mCargoIntake.setHingeSpeed(0.0);
  }
}
