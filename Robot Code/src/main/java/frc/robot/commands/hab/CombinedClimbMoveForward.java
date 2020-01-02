/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hab;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.RobotMap;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.CargoScore;
import frc.robot.subsystems.Hab;

public class CombinedClimbMoveForward extends Command {

  private Hab mHab;
  private Drive mDrive;
  private CargoIntake mCargoIntake;
  private CargoScore mCargoScore;
  

  public CombinedClimbMoveForward() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    mHab = Hab.getInstance();
    mDrive = Drive.getInstance();
    mCargoIntake = CargoIntake.getInstance();
    mCargoScore = CargoScore.getInstance();

    requires(mCargoIntake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mCargoScore.setIntakeSpeed(RobotMap.CargoIntake.ROLLER_CLIMBING_SPEED);
    mHab.holdRacks();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mCargoScore.setIntakeSpeed(RobotMap.CargoIntake.ROLLER_CLIMBING_SPEED);
    mHab.holdRacks();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mHab.overHabPlatform(); //When we know our Centre of Gravity is over the Hab Platform
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //mCargoScore.setIntakeSpeed(0.0);
    mHab.holdRacks();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mCargoScore.setIntakeSpeed(0.0);
    mHab.holdRacks();
  }
}