/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.cargo.CargoIntakeReset;
import frc.robot.commands.cargo.CargoScoreSetToZero;
import frc.robot.commands.drive.DriveResetAll;
import frc.robot.commands.elevator.ElevatorFindZero;
import frc.robot.commands.hab.HabDeactivatePTO;
import frc.robot.commands.hatch.HatchIntakeRaise;
import frc.robot.commands.hatch.HatchScoreExpanderPush;
import frc.robot.commands.hatch.HatchScoreSliderRetract;

public class ResetAll extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ResetAll() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    addParallel(new CargoIntakeReset());
    addParallel(new CargoScoreSetToZero());
    addParallel(new DriveResetAll());
    addParallel(new ElevatorFindZero());
    addParallel(new HabDeactivatePTO());
    addParallel(new HatchIntakeRaise());
    addParallel(new HatchScoreSliderRetract());
    addSequential(new HatchScoreExpanderPush());


  }
}
