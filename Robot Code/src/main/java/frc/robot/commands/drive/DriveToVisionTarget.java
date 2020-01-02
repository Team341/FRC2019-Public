/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.awt.Point;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drive;
import frc.robot.tracking.GoalTracking;
import frc.robot.utilities.DaisyMath;

public class DriveToVisionTarget extends Command {

  private Drive mDrive = Drive.getInstance();

  private Point centrePoint;
  private Point targetCentre;
  private double targetYaw;

  private double kPTurn = 0.01;
  private double kITurn = 0.05;
  private double kDTurn = 0.005;//5;

  private double kPSpeed = -0.4;
  private double kISpeed = 0.0;//5;
  private double kDSpeed = 0.0;//5;

  private double kPYaw = -0.6;
  private double kIYaw = 0.5;
  private double kDYaw = 0.0;

  private double kPPsi = -0.0;//5; //be more aggressive
  private double kIPsi = 0.0;
  private double kDPsi = 0.0;

  private double distance;
  private double speed;
  private double turnError;
  private double turnLastError;
  private double turn;
  private double yaw;
  private double theta;
  private double psi;
  private double yawError;
  private double turnScalar;
  private double areaRatio;

  private GoalTracking goalTracker;

  public DriveToVisionTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    goalTracker = GoalTracking.getInstance();
    centrePoint = new Point(RobotMap.Vision.SCREEN_CENTRE_POINT);
    targetCentre = new Point(); //Get Point from Network Table
    targetYaw = 0.0;
    turnError = 0.0;
    distance = -1.0; //get Distance from Network Table
    speed = 0.0;
    yaw = 0.0;
    psi = 0.0;
    yawError = 0.0;
    turnScalar = 0.0;
    requires(mDrive);
    SmartDashboard.putString("DriveToVisionTarget/Check Constructor", "We made it!");
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    yaw = mDrive.getGyroYaw();
    logToDashboard();
    SmartDashboard.putString("DriveToVisionTarget/Check Initialize", "We made it!");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override

  protected void execute() {
    SmartDashboard.putString("DriveToVisionTarget/Check Execute Start", "We made it!");
    //targetCentre = new Point((int)NetworkTable.getTable("GRIP/myContoursReport").getNumber("CenterX", -1), (int)NetworkTable.getTable("GRIP/myContoursReport").getNumber("CenterY", -1)); //Get Point from Network Table
    //distance = NetworkTable.getTable("GRIP/myContoursReport").getNumber("Distance", -1.0);
    kPYaw = SmartDashboard.getNumber("DriveToVisionTarget/kPYaw", kPYaw);
    kPPsi = SmartDashboard.getNumber("DriveToVisionTarget/kPPsi", kPPsi);
    //theta = NetworkTable.getTable("GRIP/myContoursReport").getNumber("theta", theta);
    //psi = NetworkTable.getTable("GRIP/myContoursReport").getNumber("Height Ratio", psi);

    yaw = mDrive.getGyroYaw();
    
    
    turnLastError = turnError;
/*
    if(hasTarget()) {
      

      turnError = centrePoint.getX() - targetCentre.getX();

      turn = DaisyMath.minmax(turnError * kPTurn, -0.8 , 0.8) + kDTurn * (turnError - turnLastError) + kPPsi * (1.0 - psi);
      SmartDashboard.putNumber("DriveToTarget/turnCommand", turn);
      speed = DaisyMath.minmax(kPSpeed * distance, -0.3, 0.3);
      
      //turn = kPYaw * (theta - yaw) + kPPsi * (1.0 - psi);
      System.out.println("Running Auto Target");
      
    } else {
      speed = 0.0;
      turn = 0.0;
    }

    */

    SmartDashboard.putString("DriveToVisionTarget/Check Execute Before Goal Tracking", "We made it!");
    if(goalTracker.hasBestTrack()) {
      SmartDashboard.putString("DriveToVisionTarget/Check If Statement", "We made it to if statement!");
      distance = goalTracker.getTargetRange();
      areaRatio = goalTracker.getAreaRatio();
      turnScalar = RobotMap.Vision.GOAL_DISTANCE_SCALAR_THRESHOLD - (RobotMap.Vision.GOAL_DISTANCE_SCALAR_THRESHOLD - distance);
      turnError = goalTracker.getTx();//goalTracker.getAngleToTargetInRadians() - Math.toRadians(yaw);
      turn = turnError * kPTurn + ((Math.toRadians(yaw) - Math.toRadians(turnError))) * kDTurn; //+ ((1.0 - areaRatio) * turnScalar) * kDTurn;// + (1.0 - psi) * kPPsi;
      turn = -1.0 * (DaisyMath.minmax(turn, -0.5, 0.5));
    } else {
      distance = 0;
      turn = 0;
    }
    SmartDashboard.putString("DriveToVisionTarget/Check Execute After Goal Tracking", "We made it!");
    if(goalTracker.getTargetRange() > RobotMap.Vision.TRACK_RANGE_MEDIUM && goalTracker.hasBestTrack()) { //Brendan
      speed = -1.0 * (DaisyMath.minmax(kPSpeed * distance, -0.3, 0.3));

    } else {
      speed = -1.0 * (DaisyMath.minmax(kPSpeed * distance, -0.1, 0.1));
    } 

    mDrive.setSpeedTurn(speed, turn);

    logToDashboard();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return distance < RobotMap.Vision.DISTANCE_OFFSET;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //mDrive.setSpeedTurn(0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //mDrive.setSpeedTurn(0.0, 0.0);
  }

  @Override
  public boolean isInterruptible() {
    return false;
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("DriveToVisionTarget/Turn Error", turnError);
    SmartDashboard.putNumber("DriveToVisionTarget/Turn Error * kPTurn", turnError * kPTurn);
    SmartDashboard.putNumber("DriveToVisionTarget/Turn", turn);
    SmartDashboard.putNumber("DriveToVisionTarget/kPTurn", kPTurn);
    SmartDashboard.putNumber("DriveToVisionTarget/Distance * kPSpeed", distance * kPSpeed);
    SmartDashboard.putNumber("DriveToVisionTarget/Speed", speed);
    SmartDashboard.putNumber("DriveToVisionTarget/kPYaw", kPYaw);
    SmartDashboard.putNumber("DriveToVisionTarget/kPPsi", kPPsi);
  }

  public static boolean hasTarget() {
    return NetworkTable.getTable("limelight").getDouble("tv", 0) > 0;//NetworkTable.getTable("GRIP/myContoursReport/").getNumber("Number of Targets", -1) > 0;
  }
}
