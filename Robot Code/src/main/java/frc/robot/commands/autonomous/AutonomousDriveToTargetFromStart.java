/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.drive.DriveTurnToFindTarget;

public class AutonomousDriveToTargetFromStart extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutonomousDriveToTargetFromStart() {
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

    //addSequential(new AutonomousDrive(RobotMap.Drive.AUTO_BACKWARD_SPEED, 0.0, RobotMap.Drive.AUTONOMOUS_DISTANCE_TO_DRIVE));
    addSequential(new DrivePath("output/Unnamed", false));
    //addSequential(new DriveTurnToFindTarget(1.0));
    //addSequential(OI.getInstance().getDriveToVisionTarget());
  }
}
