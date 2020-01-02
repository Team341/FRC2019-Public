/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.DaisyMath;

public class ElevatorManualControl extends Command {

  Elevator mElevator;
  double pos = 0.0;

  public ElevatorManualControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    mElevator = Elevator.getInstance();

    requires(mElevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mElevator.setSpeed(RobotMap.Elevator.ELEVATOR_MANUAL_CONTROL_SCALAR * OI.getInstance().mOperator2RightStickY);
    pos = mElevator.getPosition();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double speed = RobotMap.Elevator.ELEVATOR_MANUAL_CONTROL_SCALAR * OI.getInstance().mOperator2RightStickY;
    SmartDashboard.putNumber("Elevator/SetSpeedManual", speed);
    
   if(Math.abs(speed) < 0.1) {
     mElevator.setPosition(pos);
     //System.out.println("holding ele pos");
    } else {
      if(CargoIntake.getInstance().getLeftPosition() < RobotMap.CargoIntake.HINGE_TRANSIT_POSITION) {
        CargoIntake.getInstance().setPosition(RobotMap.CargoIntake.HINGE_TRANSIT_POSITION);
      }
      speed = DaisyMath.minmax(speed, -0.25, 1.0);
      mElevator.setSpeed(speed);
      pos = mElevator.getPosition();
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
    mElevator.setSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}