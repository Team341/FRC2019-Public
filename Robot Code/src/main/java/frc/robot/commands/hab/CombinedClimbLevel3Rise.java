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

public class CombinedClimbLevel3Rise extends Command {

  private Hab mHab;
  private Drive mDrive;
  private CargoIntake mCargoIntake;

  private double kPHinge = 0.1;

  public CombinedClimbLevel3Rise() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    mHab = Hab.getInstance();
    mDrive = Drive.getInstance();
    mCargoIntake = CargoIntake.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mHab.activatePTO();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mCargoIntake.setSmartPosition(RobotMap.CargoIntake.HINGE_CLIMB_POSITION);

    double rkHt = Math.max(RobotMap.CargoIntake.INTAKE_HEIGHT_LEVEL_3_START - mCargoIntake.getFrontWheelHeight(), 0);
    mHab.setRackHeight(rkHt);

    /*
    mHab.extendRacks();
    mCargoIntake.setFrontWheelHeight(mHab.getRackHeight());
    */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mHab.getRackHeight() >= RobotMap.Hab.PLATFORM_3_HEIGHT + RobotMap.Hab.PLATFORM_CLIMB_TOLERANCE;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mHab.holdRacks();
    mCargoIntake.holdPosition();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mHab.holdRacks();
    mCargoIntake.holdPosition();
  }
}