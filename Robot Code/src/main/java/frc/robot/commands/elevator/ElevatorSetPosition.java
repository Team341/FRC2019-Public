/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchIntake;
import frc.robot.subsystems.HatchScore;
import frc.robot.subsystems.CargoIntake;

public class ElevatorSetPosition extends Command {

  private Elevator mElevator = Elevator.getInstance();
  private CargoIntake mCargoIntake = CargoIntake.getInstance();
  private double height;
  private boolean isCargo;
  private int count, runCount;

  public ElevatorSetPosition(double goalPos, boolean isCargo) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    height = goalPos;
    this.isCargo = isCargo;
    count = -1;
    runCount = -1;
    requires(mElevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(mCargoIntake.getLeftPosition() < RobotMap.CargoIntake.HINGE_TRANSIT_POSITION) {
      mCargoIntake.setPosition(RobotMap.CargoIntake.HINGE_TRANSIT_POSITION);
    }
    count = 0;
    runCount = 0;
    mElevator.setPosition(height);
    
    if(!isCargo) {
      HatchScore.getInstance().extendSlider();
    } else {
      HatchScore.getInstance().retractSlider();
    }
    
    SmartDashboard.putNumber("Elevator/ElevatorSetPosition", height);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //mElevator.setPosition(height);
    if(mCargoIntake.getLeftPosition() < RobotMap.CargoIntake.HINGE_TRANSIT_POSITION){
      mCargoIntake.setPosition(RobotMap.CargoIntake.HINGE_TRANSIT_POSITION);
    }
    
    if(Math.abs(mElevator.getPosition() - height) < RobotMap.Elevator.ELEVATOR_POSITION_TOLERANCE) {
       if(height <= 500 && !mElevator.getLowerLimitSwitch()) {
          mElevator.setSpeed(-0.2);
      } else {
         count++;
      }
    } else {
      count = 0;
    }
    
    /*
    if(runCount == 5) {
      HatchIntake.getInstance().setIntakeSpeed(0.0);
    }
    */
    runCount++;
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return count > 5 || runCount > 62;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putNumber("Elevator/ElevatorSetPosition", 0.0);
    mElevator.setSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    SmartDashboard.putNumber("Elevator/ElevatorSetPosition", 0.0);
  }
}
