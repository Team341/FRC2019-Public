/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LEDLightUp;
import frc.robot.commands.cargo.CargoBothReverseIntake;
import frc.robot.commands.cargo.CargoBothRunIntake;
import frc.robot.commands.cargo.CargoIntakeManualControl;
import frc.robot.commands.cargo.CargoIntakeSetPosition;
import frc.robot.commands.cargo.CargoScoreShoot;
import frc.robot.commands.cargo.IntakeCargo;
import frc.robot.commands.drive.DriveResetEncoders;
import frc.robot.commands.drive.DriveSpeed;
import frc.robot.commands.drive.DriveToVisionTarget;
import frc.robot.commands.drive.DriveTurnDampeningOn;
import frc.robot.commands.elevator.ElevatorManualControl;
import frc.robot.commands.elevator.ElevatorSetPosition;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.commands.hab.HabActivatePTO;
import frc.robot.commands.hab.HabClimbLevel2;
import frc.robot.commands.hab.HabClimbLevel3;
import frc.robot.commands.hab.HabDeactivatePTO;
import frc.robot.commands.hab.HabFinished;
import frc.robot.commands.hatch.AcquireHatchGround;
import frc.robot.commands.hatch.CancelAcquireHatchGround;
import frc.robot.commands.hatch.HatchScoreExpanderPop;
import frc.robot.commands.hatch.HatchScoreExpanderPush;
import frc.robot.commands.hatch.HatchScoreSliderExtend;
import frc.robot.commands.hatch.HatchScoreSliderRetract;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.CargoScore;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hab;
import frc.robot.subsystems.HatchIntake;
import frc.robot.subsystems.HatchScore;
import frc.robot.subsystems.LEDs;
import frc.robot.triggers.CargoIntakeManualOverride;
import frc.robot.triggers.DriveResetOnImpact;
import frc.robot.triggers.ElevatorManualOverride;
import frc.robot.triggers.ElevatorVoltageTrigger;
import frc.robot.triggers.RunCargoBothIntake;
import frc.robot.triggers.RunCargoIntakeReverse;
import frc.robot.triggers.RunDriveToVisionTarget;
import frc.robot.triggers.RunHatchScore;
import frc.robot.triggers.VisionHasHatch;
import frc.robot.triggers.VisionTargetTrigger;
import frc.robot.triggers.VisionTargetTriggerClose;
import frc.robot.triggers.RunCargoScoreShoot;
//import frc.robot.commands.DriveFindVisionTarget;
//import frc.robot.commands.DriveToVisionTarget;
//import frc.robot.commands.ElevatorSetPosition;
import frc.robot.utilities.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public static OI instance = null;

  public static OI getInstance() {
    if(instance == null) {
      instance = new OI();
    }
    return instance;
  }

  
  private Drive mDrive = Drive.getInstance();
  private Elevator mElevator = Elevator.getInstance();
  private CargoIntake mCargoIntake = CargoIntake.getInstance();
  private CargoScore mCargoScore = CargoScore.getInstance();
  private HatchScore mHatchScore = HatchScore.getInstance();
  private HatchIntake mHatchIntake = HatchIntake.getInstance();
  private Hab mHab = Hab.getInstance();
  private LEDs mLEDs = LEDs.getInstance();
  

  private XboxController mDriverController = new XboxController(RobotMap.Controller.DRIVER_CONTROLLER_PORT);
  private Joystick mOperatorController1 = new Joystick(RobotMap.Controller.OPERATOR_CONTROLLER_1_PORT);
  private Joystick mOperatorController2 = new Joystick(RobotMap.Controller.OPERATOR_CONTROLLER_2_PORT);

  private Button mDriverButtonA = mDriverController.ButtonA;
  private Button mDriverButtonB = mDriverController.ButtonB;
  private Button mDriverButtonX = mDriverController.ButtonX;
  private Button mDriverButtonY = mDriverController.ButtonY;
  private Button mDriverButtonLB = mDriverController.BumperLeft;
  private Button mDriverButtonRB = mDriverController.BumperRight;


  private Button mRunDriveToVisionTarget, mVisionTargetTrigger, mVisionHasHatch, mDriveResetOnImpact,
                mRunCargoBothIntake, mRunCargoIntakeReverse, mElevatorManualOverride,
                mVisionTargetTriggerClose, mCargoIntakeManualOverride, mElevatorVoltageTrigger, mRunCargoScoreShoot, mRunHatchScore;


  private DriveToVisionTarget mDriveToVisionTarget = null;// = new DriveToVisionTarget();
  public AcquireHatchGround mAcquireHatchGround = null;
  
  //Operator Controller 1 buttons
  private Button mCargoHingeLoad = new JoystickButton(mOperatorController1, RobotMap.Controller.CARGO_HINGE_LOAD_PORT);
  private Button mCargoHingeTransit = new JoystickButton(mOperatorController1, RobotMap.Controller.CARGO_HINGE_TRANSIT_PORT);
  private Button mCargoHingeStow = new JoystickButton(mOperatorController1, RobotMap.Controller.CARGO_HINGE_STOW_PORT);
  private Button mHabClimbLevel2 = new JoystickButton(mOperatorController1, RobotMap.Controller.HAB_CLIMB_LEVEL_2_PORT);
  private Button mHabClimbLevel3 = new JoystickButton(mOperatorController1, RobotMap.Controller.HAB_CLIMB_LEVEL_3_PORT);
  private Button mElevatorCargoLow = new JoystickButton(mOperatorController1, RobotMap.Controller.ELEVATOR_CARGO_LOW_PORT);
  private Button mElevatorCargoHold = new JoystickButton(mOperatorController1, RobotMap.Controller.ELEVATOR_CARGO_HOLD_PORT);
  private Button mElevatorCargoMed = new JoystickButton(mOperatorController1, RobotMap.Controller.ELEVATOR_CARGO_MED_PORT);
  private Button mElevatorCargoHigh = new JoystickButton(mOperatorController1, RobotMap.Controller.ELEVATOR_CARGO_HIGH_PORT);

  //Operator Controller 2 buttons
  private Button mHatchExpanderIn = new JoystickButton(mOperatorController2, RobotMap.Controller.HATCH_EXPANDER_IN_PORT);
  private Button mHatchSliderOut = new JoystickButton(mOperatorController2, RobotMap.Controller.HATCH_SLIDER_OUT_PORT);
  private Button mHatchScoopLoad = new JoystickButton(mOperatorController2, RobotMap.Controller.HATCH_SCOOP_LOAD_PORT);
  private Button mCargoIntakeReverse = new JoystickButton(mOperatorController2, RobotMap.Controller.CARGO_INTAKE_REVERSE_PORT);
  private Button mCargoIntakeRun = new JoystickButton(mOperatorController2, RobotMap.Controller.CARGO_INTAKE_RUN_PORT);
  private Button mCargoScoreRun = new JoystickButton(mOperatorController2, RobotMap.Controller.CARGO_SCORE_RUN_PORT);
  private Button mElevatorBottom = new JoystickButton(mOperatorController2, RobotMap.Controller.ELEVATOR_BOTTOM_PORT);
  private Button mElevatorHatchLow = new JoystickButton(mOperatorController2, RobotMap.Controller.ELEVATOR_HATCH_LOW_PORT);
  private Button mElevatorHatchMed = new JoystickButton(mOperatorController2, RobotMap.Controller.ELEVATOR_HATCH_MED_PORT);
  private Button mElevatorHatchHigh = new JoystickButton(mOperatorController2, RobotMap.Controller.ELEVATOR_HATCH_HIGH_PORT);
  //private Button mResetAll = new JoystickButton(mOperatorController2, RobotMap.Controller.RESET_ALL_PORT);

  public double mDriverLeftStickX = 0.0;
  public double mDriverLeftStickY = 0.0;
  public double mDriverRightStickX = 0.0;
  public double mDriverRightStickY = 0.0;

  public double mDriverLeftTrigger = 0.0;
  public double mDriverRightTrigger = 0.0;

  public double mOperator1LeftStickX = 0.0;
  public double mOperator1LeftStickY = 0.0; 
  public double mOperator2RightStickX = 0.0;
  public double mOperator2RightStickY = 0.0;

  private double rumble = 0.0;

  int count = 0; //allows triggers using OI to instantiate on first run

  public OI() {
    count = 0;

    
    //Triggers
    mVisionTargetTrigger = new VisionTargetTrigger();
    mVisionTargetTrigger.whileHeld(new LEDLightUp(RobotMap.LEDS.VISION_HATCH));

    mVisionTargetTriggerClose = new VisionTargetTriggerClose();
    mVisionTargetTriggerClose.whileHeld(new LEDLightUp(RobotMap.LEDS.VISION_BALL));

    mVisionHasHatch = new VisionHasHatch();
    mVisionHasHatch.whenPressed(new LEDLightUp(RobotMap.LEDS.CARGO_HATCH, true));

    mDriveResetOnImpact = new DriveResetOnImpact();
    mDriveResetOnImpact.whenPressed(new DriveResetEncoders());

    mElevatorVoltageTrigger = new ElevatorVoltageTrigger();
    mElevatorVoltageTrigger.whileHeld(new LEDLightUp(RobotMap.LEDS.ELEVATOR_VOLTAGE, true));

    mRunDriveToVisionTarget = new RunDriveToVisionTarget();
    mRunCargoBothIntake = new RunCargoBothIntake();
    mRunCargoIntakeReverse = new RunCargoIntakeReverse();
    mElevatorManualOverride = new ElevatorManualOverride();
    mCargoIntakeManualOverride = new CargoIntakeManualOverride();
    mRunCargoScoreShoot = new RunCargoScoreShoot();
    mRunHatchScore = new RunHatchScore();

    mAcquireHatchGround = new AcquireHatchGround();

    //Driver Controller Buttons
    mDriverButtonA.whileHeld(new DriveSpeed(-0.2));

    mDriverButtonB.whenPressed(new HabActivatePTO());

    mDriverButtonX.whenPressed(new HabDeactivatePTO());

    mDriverButtonY.whileHeld(new DriveSpeed(0.2));

    //mDriverButtonRB.whileHeld(new HabManualClimb());
    mDriverButtonLB.whileHeld(new DriveTurnDampeningOn());
    mDriverButtonRB.whileHeld(getDriveToVisionTarget());
      
    //Operator Controller 1 Buttons
    mCargoHingeStow.whenPressed(new CargoIntakeSetPosition(RobotMap.CargoIntake.HINGE_STOW_POSITION));
    mCargoHingeTransit.whenPressed(new CargoIntakeSetPosition(RobotMap.CargoIntake.HINGE_TRANSIT_POSITION));
    mCargoHingeLoad.whenPressed(new CargoIntakeSetPosition(RobotMap.CargoIntake.HINGE_LOAD_POSITION));
    //mCargoHingeLoad.whileHeld(new IntakeCargo());
    mCargoIntakeReverse.whileHeld(new CargoBothReverseIntake());
    mCargoIntakeRun.whileHeld(new IntakeCargo());
    mCargoScoreRun.whileHeld(new CargoScoreShoot());
    mElevatorCargoLow.whenPressed(new ElevatorSetPosition(RobotMap.Elevator.ELEVATOR_CARGO_POSITION_BOTTOM, true));
    mElevatorCargoHold.whenPressed(new ElevatorSetPosition(RobotMap.Elevator.ELEVATOR_CARGO_POSITION_HOLD, true));
    mElevatorCargoMed.whenPressed(new ElevatorSetPosition(RobotMap.Elevator.ELEVATOR_CARGO_POSITION_MIDDLE, true));
    mElevatorCargoHigh.whenPressed(new ElevatorSetPosition(RobotMap.Elevator.ELEVATOR_CARGO_POSITION_TOP, true));

    //Operator Controller 2 Buttons
    mHatchExpanderIn.whileHeld(new HatchScoreExpanderPush());
    mHatchExpanderIn.whenReleased(new HatchScoreExpanderPop());
    //mHatchExpanderIn.whenReleased(new HatchScoreSliderRetract());
    //mHabClimbLevel2.whileHeld(new HabClimbLevel2());
    mHabClimbLevel2.whileHeld(new HabClimbLevel2());
    mHabClimbLevel2.whenReleased(new HabFinished());
    mHabClimbLevel3.whileHeld(new HabClimbLevel3());
    mHabClimbLevel3.whenReleased(new HabFinished());
        //mHabClimbLevel3.whileHeld(new HatchScoreExpanderPush());
    //mHatchScoopLoad.whenPressed(new HatchIntakeLower());
    mHatchScoopLoad.whenPressed(mAcquireHatchGround);
    mHatchScoopLoad.whenReleased(new CancelAcquireHatchGround());
    mHatchSliderOut.whenPressed(new HatchScoreSliderExtend());
    mHatchSliderOut.whenReleased(new HatchScoreSliderRetract());
    mElevatorBottom.whenPressed(new ZeroElevator());
    mElevatorHatchLow.whenPressed(new ElevatorSetPosition(RobotMap.Elevator.ELEVATOR_HATCH_POSITION_LOW, false));
    mElevatorHatchMed.whenPressed(new ElevatorSetPosition(RobotMap.Elevator.ELEVATOR_HATCH_POSITION_MIDDLE, false));
    mElevatorHatchHigh.whenPressed(new ElevatorSetPosition(RobotMap.Elevator.ELEVATOR_HATCH_POSITION_TOP, false));
    //mResetAll.whenPressed(new ResetAll());
  }

  public void run() {
    count++;

    if(count == 1) {
      mRunCargoIntakeReverse.whileHeld(new CargoBothReverseIntake());
      mRunCargoBothIntake.whileHeld(new CargoBothRunIntake());
      //mRunDriveToVisionTarget.whileHeld(getDriveToVisionTarget());
      mElevatorManualOverride.whileHeld(new ElevatorManualControl());
      mCargoIntakeManualOverride.whileHeld(new CargoIntakeManualControl());
      mRunCargoScoreShoot.whileHeld(new CargoScoreShoot());
      mRunHatchScore.whileHeld(new HatchScoreExpanderPush());
      mRunHatchScore.whenReleased(new HatchScoreExpanderPop());
      //mRunHatchScore.whenReleased(new HatchScoreSliderRetract());
    }

    mDriverLeftStickX = scaleInput(-1.0 * mDriverController.getDeadbandedLeftXAxis(RobotMap.Controller.DEADBAND), RobotMap.Controller.DEADBAND);
    mDriverLeftStickY = scaleInput(-1.0 * mDriverController.getDeadbandedLeftYAxis(RobotMap.Controller.DEADBAND), RobotMap.Controller.DEADBAND);
    mDriverRightStickX = scaleInput(-1.0 * mDriverController.getDeadbandedRightXAxis(RobotMap.Controller.DEADBAND), RobotMap.Controller.DEADBAND);
    mDriverRightStickY = scaleInput(-1.0 * mDriverController.getDeadbandedRightYAxis(RobotMap.Controller.DEADBAND), RobotMap.Controller.DEADBAND);

    mOperator1LeftStickX = -1.0 * mOperatorController1.getY();
    mOperator1LeftStickY = 1.0 * mOperatorController1.getX();

    mOperator2RightStickX = -1.0 * mOperatorController2.getY();
    mOperator2RightStickY = 1.0 * mOperatorController2.getX();

    mDriverLeftTrigger = mDriverController.getRawAxis(XboxController.Axis_LeftTrigger);
    mDriverRightTrigger = mDriverController.getRawAxis(XboxController.Axis_RightTrigger);

    SmartDashboard.putNumber("Drive/DriverLeftTrigger", mDriverLeftTrigger);
  }

  public void rumbleDriverController(double rumble) {
    mDriverController.rumble(rumble, rumble);
  }

  public double scaleInput(double in, double deadband) {
    if(Math.abs(in) < deadband) {
      return 0.0;
    } else {
      return Math.signum(in) * (Math.abs(in) - deadband) / (1.0 - deadband);
    }
  }

  public void logToDashboard() {
    
    mLEDs.logToDashboard();
    mDrive.logToDashboard();
    mCargoIntake.logToDashboard();
    
    mElevator.logToDashboard();
    
    mHab.logToDashboard();
    mHatchIntake.logToDashboard();
    mHatchScore.logToDashboard();
    mCargoScore.logToDashboard();
  }

  public DriveToVisionTarget getDriveToVisionTarget() {
    if(mDriveToVisionTarget == null) {
      mDriveToVisionTarget = new DriveToVisionTarget();
    }
    return mDriveToVisionTarget;
  }

  public boolean getVisionButton() {
    return mDriverButtonRB.get();
  }
}