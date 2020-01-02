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
import frc.robot.subsystems.CargoScore;
import frc.robot.subsystems.HatchScore;

public class CargoScoreShoot extends Command {

  private CargoScore mCargoScore = CargoScore.getInstance();

  public CargoScoreShoot() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(mCargoScore);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mCargoScore.setSpeed(RobotMap.CargoScore.SCORE_SPEED);
    // CargoIntake.getInstance().runConveyor();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mCargoScore.setSpeed(0.0);
    CargoScore.getInstance().setConveyorSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
