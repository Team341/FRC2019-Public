/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.cargo.CargoIntakeSetPosition;
import frc.robot.commands.elevator.ElevatorSetPosition;
import frc.robot.commands.hatch.HatchIntakeRaise;
import frc.robot.commands.hatch.HatchScoreExpanderPush;
import frc.robot.commands.hatch.HatchScoreSliderRetract;

public class GoDefend extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GoDefend() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()c     
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    
    addParallel(new HatchIntakeRaise());
    addParallel(new ElevatorSetPosition(RobotMap.Elevator.ELEVATOR_POSITION_BOTTOM, true));
    addParallel(new HatchScoreExpanderPush());
    addSequential(new HatchScoreSliderRetract());
    addSequential(new CargoIntakeSetPosition(RobotMap.CargoIntake.HINGE_TRANSIT_POSITION));
  }
}
