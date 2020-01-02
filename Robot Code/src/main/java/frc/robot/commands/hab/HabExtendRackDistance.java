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
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hab;

public class HabExtendRackDistance extends Command {

  private Hab mHab;
  private Drive mDrive;
  private CargoIntake mIntake;

  private double distance, kP = 0.0, direction;

  private boolean done = false;

  public HabExtendRackDistance(double dist) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    distance = Math.abs(dist * RobotMap.Hab.DRIVE_ROTATIONS_TO_RACK_HEIGHT);
    direction = Math.signum(dist);

    mHab = Hab.getInstance();
    mDrive = Drive.getInstance();
    mIntake = CargoIntake.getInstance();

    requires(mHab);
    requires(Drive.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mHab.activatePTO();
    done = false;
    //mHab.setRackHeight(distance); // * RobotMap.Hab.DRIVE_ROTATIONS_TO_RACK_HEIGHT);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(!mHab.getPTOState()) {
      mHab.activatePTO();
    }

    double leftSpeed = 0.9 * direction;
    double rightSpeed = -(0.9 * direction);
    double sideToSideError = Math.abs(mDrive.getLeftEncoderPosition()) - Math.abs(mDrive.getRightEncoderPosition());

    leftSpeed -= sideToSideError * kP * direction;
    rightSpeed += sideToSideError * kP * direction;

    if (Math.abs(mDrive.getLeftEncoderPosition()) >= Math.abs(distance)){
      leftSpeed = 0.0;
    }
    if (Math.abs(mDrive.getRightEncoderPosition()) >= Math.abs(distance)){
      rightSpeed = 0.0;
    }

    if(leftSpeed == 0.0 && rightSpeed == 0.0) {
      done = true;
    }
    mHab.setRackSpeed(leftSpeed, rightSpeed);

    if (!mIntake.getHingeLowerLimitSwitch()){
      mIntake.setHingeSpeed(0.5);
    } else {
      mIntake.setHingeSpeed(0.0);
    }


    //mHab.setRackHeight(distance);
    //System.out.println(Drive.getInstance().getLeftMotorVoltage() + "    " + Drive.getInstance().getRightMotorVoltage());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return done;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mHab.holdRacks();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mHab.holdRacks();
  }
}