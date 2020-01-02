/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.LEDs;
public class LEDLightUp extends Command {

  private LEDs mLED;

  private int color, count, runCount;
  private boolean blink;

  public LEDLightUp(int color, boolean blink) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.color = color;
    this.blink = blink;
    count = -1;
    runCount = -1;

    mLED = LEDs.getInstance();
  }

  public LEDLightUp(int color) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.color = color;
    this.blink = false;
    count = -1;
    runCount = -1;

    mLED = LEDs.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mLED.setColor(color);
    count = 0;
    runCount = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(blink) {
      if(count % 4 < 2) {
        mLED.setColor(color);
      } else {
        mLED.setColor(RobotMap.LEDS.NO_COLOR);
      }
      count++;
    }
    
    runCount++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return count > 12;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mLED.setColor(RobotMap.LEDS.NO_COLOR);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mLED.setColor(RobotMap.LEDS.NO_COLOR);
  }
}
