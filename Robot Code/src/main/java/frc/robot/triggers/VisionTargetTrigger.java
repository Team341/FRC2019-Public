/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import frc.robot.tracking.GoalTracking;

/**
 * Add your docs here.
 */
public class VisionTargetTrigger extends Button {
    
  public boolean get() {
    return GoalTracking.getInstance().getNumOfTargets() > 0.0;
  }   

}
