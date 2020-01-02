/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hab;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.CargoScore;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hab;

public class CombinedClimbFinishClimb extends Command {

  private Hab mHab;
  private Drive mDrive;
  private CargoIntake mCargoIntake;
  private CargoScore mCargoScore;

  public CombinedClimbFinishClimb() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    mHab = Hab.getInstance();
    mDrive = Drive.getInstance();
    mCargoIntake = CargoIntake.getInstance();
    mCargoScore = CargoScore.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //mDrive.resetEncoderPositions();
    mDrive.setSpeedTurn(RobotMap.Drive.DRIVE_CLIMB_SPEED, 0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mCargoIntake.setPosition(RobotMap.CargoIntake.HINGE_STOW_POSITION);
    mHab.retractRacks();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mDrive.getLeftEncoderPosition() > RobotMap.Drive.CLIMB_DISTANCE_TO_DRIVE;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mDrive.setSpeed(0.0, 0.0);
    mCargoScore.setIntakeSpeed(0.0);
    mCargoIntake.setHingeSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
