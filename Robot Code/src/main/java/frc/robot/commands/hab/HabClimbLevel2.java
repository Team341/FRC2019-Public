
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hab;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.cargo.CargoIntakeRunRollers;
import frc.robot.commands.cargo.CargoIntakeSetPosition;

public class HabClimbLevel2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HabClimbLevel2() {
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

    /*
    addSequential(new CombinedClimbLevel2Rise());
    addSequential(new CombinedClimbMoveForward());
    addSequential(new CombinedClimbFinishClimb());
*/
    //addSequential(new CargoIntakeSetPosition(RobotMap.CargoIntake.HINGE_CLIMB_POSITION));
    addSequential(new HabActivatePTO());
    addSequential(new CargoIntakeSetPosition(RobotMap.CargoIntake.HINGE_CLIMB_POSITION));
    // addSequential(new HabExtendRackDistance(RobotMap.Hab.PLATFORM_2_HEIGHT));
    addSequential(new CargoIntakeRunRollers(1.0));
    //addSequential(new CombinedClimbMoveForward());
    //addSequential(new CombinedClimbFinishClimb());
  }
}
