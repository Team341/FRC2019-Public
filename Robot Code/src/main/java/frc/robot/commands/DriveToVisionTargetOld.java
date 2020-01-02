/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.awt.Point;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.DaisyMath;

public class DriveToVisionTargetOld extends Command {

  private Drive mDrive = Drive.getInstance();

  private Point centrePoint;
  private Point targetCentre;
  private double targetYaw;

  private double kPTurn = -0.0075;
  private double kITurn = 0.0;
  private double kDTurn = 0.0;

  private double kPSpeed = -0.4;
  private double kISpeed = 0.0;
  private double kDSpeed = 0.0;

  private double kPYaw = -0.4;
  private double kIYaw = 0.0;
  private double kDYaw = 0.0;

  private double kPPsi = -0.8; //be more aggressive
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

  public DriveToVisionTargetOld() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    centrePoint = new Point(RobotMap.Vision.SCREEN_CENTRE_POINT);
    targetCentre = new Point(); //Get Point from Network Table
    targetYaw = 0.0;
    turnError = 0.0;
    distance = -1.0; //get Distance from Network Table
    speed = 0.0;
    yaw = 0.0;
    psi = 0.0;
    yawError = 0.0;
    requires(mDrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putNumber("DriveToVisionTarget/kPTurn", kPTurn);
    yaw = mDrive.getGyroYaw();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    targetCentre = new Point((int)NetworkTable.getTable("GRIP/myContoursReport").getNumber("CenterX", -1), (int)NetworkTable.getTable("GRIP/myContoursReport").getNumber("CenterY", -1)); //Get Point from Network Table
    distance = NetworkTable.getTable("GRIP/myContoursReport").getNumber("Distance", -1.0);
    kPTurn = SmartDashboard.getNumber("DriveToVisionTarget/kPTurn", kPTurn);
    kPPsi = SmartDashboard.getNumber("DriveToVisionTarget/kPPsi", kPPsi);
    theta = NetworkTable.getTable("GRIP/myContoursReport").getNumber("theta", theta);
    psi = NetworkTable.getTable("GRIP/myContoursReport").getNumber("Height Ratio", psi);

    yaw = mDrive.getGyroYaw();
    

    
    turnLastError = turnError;

    if(hasTarget()) {
      

      turnError = centrePoint.getX() - targetCentre.getX();

      turn = DaisyMath.minmax(turnError * kPTurn, -0.8 , 0.8) + kDTurn * (turnError - turnLastError) + kPPsi * (1.0 - psi);
      
      speed = DaisyMath.minmax(kPSpeed * distance, -0.3, 0.3);
      
      //turn = kPYaw * (theta - yaw) + kPPsi * (1.0 - psi);
      
    } else {
      speed = 0.0;
      turn = 0.0;
    }

    turn = DaisyMath.minmax(turn, -0.25, 0.25);

    mDrive.setSpeedTurn(speed, turn);

    SmartDashboard.putNumber("Turn Error", turnError);
    SmartDashboard.putNumber("Turn Error * kPTurn", turnError * kPTurn);
    SmartDashboard.putNumber("Turn", turn);
    SmartDashboard.putNumber("DriveToVisionTarget/kPTurn", kPTurn);
    SmartDashboard.putNumber("Distance * kPSpeed", distance * kPSpeed);
    SmartDashboard.putNumber("Speed", speed);

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

  public static boolean hasTarget() {
    return  NetworkTable.getTable("GRIP/myContoursReport").getNumber("CenterX", -1) != -1;
  }
}
