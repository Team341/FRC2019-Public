/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.cargo.CargoIntakeSetPosition;
import frc.robot.commands.elevator.ElevatorSetPosition;

public class IntakeCargo extends CommandGroup {
  /**
   * This command group moves the elevator and intake arm into position to acquire a cargo from the ground.
   * Once in position, the rollers will spin until a cargo is detected by the ball sensor in the cargo scorer,
   * indicating a ball has been acquired.
   */
  public IntakeCargo() {
    // Move the elevator & Cargo intake into cargo load position
    addParallel(new ElevatorSetPosition(RobotMap.Elevator.ELEVATOR_POSITION_BOTTOM, true));

    // Run the cargo intake rollers, cargo conveyor rollers full speed 
    // and cargo scorer rollers slow until the ball sensor detects a ball
    addSequential(new CargoBothRunIntake());
    addSequential(new CargoIntakeSetPosition(RobotMap.CargoIntake.HINGE_TRANSIT_POSITION));
  }
}
