/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoScore;

public class CargoBothRunIntake extends Command {

  private CargoScore mCargoScore = CargoScore.getInstance();

  private int trueCount;

  public CargoBothRunIntake() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    trueCount = 0;

    requires(mCargoScore);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mCargoScore.runConveyor();
    mCargoScore.runIntake();
    mCargoScore.intakeBall();

    trueCount = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mCargoScore.runConveyor();
    mCargoScore.runIntake();
    mCargoScore.intakeBall();
    
    if(mCargoScore.ballAcquired()) {
      trueCount++;
    } else {
      trueCount = 0;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return trueCount > 20;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mCargoScore.setConveyorSpeed(0.0);
    mCargoScore.setIntakeSpeed(0.0);
    mCargoScore.setSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
