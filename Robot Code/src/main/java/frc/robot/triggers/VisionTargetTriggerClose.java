/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.RobotMap;
import frc.robot.tracking.GoalTracking;

/**
 * Add your docs here.
 */
public class VisionTargetTriggerClose extends Button {
    
  private int hasTargetCounter = -1;
  public boolean get() {
    if (Math.abs(GoalTracking.getInstance().getTargetRange()) < RobotMap.Vision.TARGET_CLOSE_DISTANCE_THRESHOLD){
      hasTargetCounter = 5;
    }else{
      hasTargetCounter -= 1;
    }
    return hasTargetCounter >= 0;
    //return Math.abs(GoalTracking.getInstance().getTargetRange()) < RobotMap.Vision.TARGET_CLOSE_DISTANCE_THRESHOLD;
  }   

}
