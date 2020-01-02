/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.OI;

/**
 * Add your docs here.
 */
public class ElevatorManualOverride extends Button {
    
  public boolean get() {
    return Math.abs(OI.getInstance().mOperator2RightStickY) > 0.5;
  }   

}
