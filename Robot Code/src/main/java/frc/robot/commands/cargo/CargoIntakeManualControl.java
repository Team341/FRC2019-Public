/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.CargoIntake;

// Joystick 1 command
// Buttons 3-5 command
public class CargoIntakeManualControl extends Command {

  private CargoIntake mCargoIntake = CargoIntake.getInstance();

  public CargoIntakeManualControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(mCargoIntake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double speed = OI.getInstance().mOperator1LeftStickY;

    mCargoIntake.setHingeSpeed(speed * RobotMap.CargoIntake.MANUAL_CONTROL_SCALAR);

    if(Math.abs(speed) > 0.5) {
      // Change the set point, so that if after we let go of the stick it will keep holding the last position it reached. This way the operating mode of the speed controller doesn't change
      //mCargoIntake.setPosition((int) Math.round(mCargoIntake.getSetpoint() + speed * RobotMap.CargoIntake.MANUAL_CONTROL_SCALAR));
  //  } else {
  //    mCargoIntake.holdPosition();
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
    //CargoIntake.setHingeSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mCargoIntake.setHingeSpeed(0.0);
  }
}
