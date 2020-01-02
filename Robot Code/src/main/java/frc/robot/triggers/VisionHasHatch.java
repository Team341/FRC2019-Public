/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import frc.robot.subsystems.CargoScore;
import frc.robot.subsystems.HatchIntake;

/**
 * Add your docs here.
 */
public class VisionHasHatch extends Button {
    
  public boolean get() {
    return HatchIntake.getInstance().hasHatch();
  }   

}
