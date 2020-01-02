/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.CargoScore;

public class CargoIntakeRunRollers extends Command {

  private CargoIntake mCargoIntake;
  private CargoScore mCargoScore;

  private double speed;

  public CargoIntakeRunRollers(double spd) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    speed = spd;

    mCargoIntake = CargoIntake.getInstance();
    mCargoScore = CargoScore.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mCargoScore.setIntakeSpeed(speed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mCargoScore.setIntakeSpeed(speed);
    if (!mCargoIntake.getHingeLowerLimitSwitch()){
      mCargoIntake.setHingeSpeed(0.5);
    } else {
      mCargoIntake.setHingeSpeed(0.0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mCargoScore.setIntakeSpeed(0.0);
    System.out.println("Intake Wheels Stopped");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();

  }
}