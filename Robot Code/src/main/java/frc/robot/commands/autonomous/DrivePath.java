/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import java.io.IOException;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.DaisyMath;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class DrivePath extends Command {

  private Drive mDrive = Drive.getInstance();
  private String pathName;

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;

  private boolean isBackwards;

  public DrivePath(String filename, boolean isBackwards) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.pathName = filename;
    this.isBackwards = isBackwards;

    requires(mDrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mDrive.resetGyroPosition();
    try {
      Trajectory left_trajectory = PathfinderFRC.getTrajectory(pathName + ".left");
      Trajectory right_trajectory = PathfinderFRC.getTrajectory(pathName + ".right");

      m_left_follower = new EncoderFollower(left_trajectory);
      m_right_follower = new EncoderFollower(right_trajectory);

      if(isBackwards) {
        initBackwards();
      } else {
        initForwards();
      }
    } catch (IOException e) {
		  e.printStackTrace();
	  }
  }

  private void initForwards() throws IOException {
    m_left_follower.configureEncoder((int)(mDrive.getLeftEncoderPosition() * 1023), 1024, 1.0 / Math.PI);
    // You must tune the PID values on the following line!
    m_left_follower.configurePIDVA(0.2, 0.0, 0.0, 1 / RobotMap.Drive.MAX_VELOCITY, 0);

    m_right_follower.configureEncoder((int)(mDrive.getRightEncoderPosition() * 1023), 1024, 1.0 / Math.PI);
    // You must tune the PID values on the following line!
    m_right_follower.configurePIDVA(0.2, 0.0, 0.0, 1 / RobotMap.Drive.MAX_VELOCITY, 0);
  }

  private void initBackwards() throws IOException {
    m_left_follower.configureEncoder((int)(mDrive.getRightEncoderPosition() * 1023), 1024, 1);
    // You must tune the PID values on the following line!
    m_left_follower.configurePIDVA(0.2, 0.0, 0.0, 1 / RobotMap.Drive.MAX_VELOCITY, 0);

    m_right_follower.configureEncoder((int)(mDrive.getLeftEncoderPosition() * 1023), 1024, 1);
    // You must tune the PID values on the following line!
    m_right_follower.configurePIDVA(0.2, 0.0, 0.0, 1 / RobotMap.Drive.MAX_VELOCITY, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(isBackwards) {
      exeBackwards();
    } else {
      exeForwards();
    }

    SmartDashboard.putNumber("DrivePath/SegmentX", m_left_follower.getSegment().x);
    SmartDashboard.putNumber("DrivePath/SegmentVelocity", m_left_follower.getSegment().velocity);
  }

  private void exeForwards() {
    double left_speed = DaisyMath.minmax(m_left_follower.calculate((int)(mDrive.getLeftEncoderPosition() * 1023)), -1.0, 1.0);
    double right_speed = DaisyMath.minmax(m_right_follower.calculate((int)(mDrive.getRightEncoderPosition() * 1023)), -1.0, 1.0);
    double heading = mDrive.getGyroYaw() + 180.0;
    double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
    double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
    double turn =  0.0 * (-1.0/80.0) * heading_difference;
    mDrive.setSpeed(0.3 * (left_speed + turn), 0.3 * (right_speed - turn));
  }

  private void exeBackwards() {
    double left_speed = DaisyMath.minmax(m_left_follower.calculate((int)mDrive.getRightEncoderPosition()), -1.0, 1.0);
    double right_speed = DaisyMath.minmax(m_right_follower.calculate((int) (mDrive.getLeftEncoderPosition())), -1.0, 1.0);
    double heading = mDrive.getGyroYaw();// + 180;
    double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
    double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
    double turn =  0.8 * (-1.0/80.0) * heading_difference;
    mDrive.setSpeed(-0.3 * (right_speed - turn), -0.3 * (left_speed + turn));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (m_left_follower.isFinished() || m_right_follower.isFinished());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mDrive.setSpeed(0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
