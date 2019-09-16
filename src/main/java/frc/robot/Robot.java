/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.Logger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.BallManipulator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.HatchManipulator;
import frc.robot.subsystems.LiftGear;
import frc.robot.subsystems.LimeLight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static final boolean DEBUG = true;

  public static Arm arm;             
  public static Elevator elevator;
  public static BallManipulator ballManipulator;
  public static HatchManipulator hatchManipulator;
  public static LiftGear liftGear;
  public static DriveTrain driveTrain;
  //public static CameraServer cameraServer; uncomment if camera exists
  public static Logger logger;
  public static PowerDistributionPanel pdp;
  public static CANifier armCanifier;
  public static CANifier elevatorCanifier;
  public static LimeLight limeLight;

  public static OI oi;

  private double lastTime = 0; // millis seconds
  private static double deltaTime = 0; // seconds
  private int loopCount = 0, executeLoopCount = 30;

  Command autonomousCommand;
  SendableChooser<AutonomousOptions> chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    //cameraServer = CameraServer.getInstance(); uncomment if camera exists
    //cameraServer.startAutomaticCapture(0); uncomment if camera exists
    arm = new Arm();
    ballManipulator = new BallManipulator();
    pdp = new PowerDistributionPanel();
    hatchManipulator = new HatchManipulator();
    elevator = new Elevator();
    liftGear = new LiftGear();
    driveTrain = new DriveTrain();
    limeLight = new LimeLight();

    // Create the logging instance so we can use it for tuning the PID subsystems
    logger = Logger.getInstance();

    // Instanciate the Power Distribution Panel so that we can get the currents
    // however, we need to clear the faults so that the LEDs on the PDP go green.
    // I can never (and I have tried) find the source of the warnings that cause
    // the LED's to be Amber.
    pdp = new PowerDistributionPanel();
    pdp.clearStickyFaults();

    chooser.setDefaultOption("Do Nothing", AutonomousOptions.DoNothing);
    chooser.addOption("Left Cargo Ship", AutonomousOptions.LeftCargoShip);
    chooser.addOption("Right Cargo Ship", AutonomousOptions.RightCargoShip);
    chooser.addOption("Left Bottom Rocket", AutonomousOptions.LeftBottomRocket);
    chooser.addOption("Right Bottom Rocket", AutonomousOptions.RightBottomRocket);
    chooser.addOption("Hab 1", AutonomousOptions.DriveOffHab1);
    chooser.addOption("Hab 2", AutonomousOptions.DriveOffHab2);
    chooser.addOption("TestMotionProfile", AutonomousOptions.TestMotionProfile);
    SmartDashboard.putData("Auto mode", chooser);

    // Make these last so to chase away the dreaded null subsystem errors!
    oi = new OI();
  }

  @Override
  public void robotPeriodic() {
    if (loopCount > executeLoopCount) {
      Robot.elevator.getElevatorData();
      Robot.arm.getArmData();
      updateSmartDashboard();
      loopCount = 0;
    } else {
      loopCount++;
    }

    deltaTime = (System.currentTimeMillis() - lastTime) / 1000;
    lastTime = System.currentTimeMillis();

    boolean safe = elevator.GetPosition() > RobotMap.Values.armSwitchHeight + 2000;
    SmartDashboard.putBoolean("wtf/Safe?", safe);
  }

  @Override
  public void disabledInit() {
    driveTrain.setCoast(); // So the drivers don't want to kill us ;)
    arm.SetIdleCoastMode();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    arm.SetIdleBrakeMode();
    Scheduler.getInstance().add(new LockArm());

    // Get the autonomous chooser option
    AutonomousOptions autonomousOption = chooser.getSelected();

    // An autonomous command must be set as a result of this activity

    if (autonomousOption == null) {
      // If it is null for some reason, do nothing. This should not happen and maybe
      // should be logged...
      autonomousCommand = new AutoDoNothing();
    } else {
      // You can call cameraControlStateMachine.autoLockRight(), 
      // cameraControlStateMachine.autoLockLeft(), or cameraControlStateMachine.autoLock()
      // from your commands if you want to sandwich in vision autolocking after initial
      // motion profile driving. Once initiated, then in a subsequent command to perform
      // auto-drive based on vision feedback, use if (cameraControlStateMachine.getState() == CameraControlStateMachine.State.AutoLocked)
      // conditional to determine if target is locked. Finally, use cameraControlStateMachine.getSelectedTarget() to get information
      // about target. This function goes to network tables for you and gets the information about the lock on target
      // as documented here: https://github.com/Team997Coders/2019DSHatchFindingVision/tree/master/CameraVision
      switch(autonomousOption) {
        case LeftCargoShip:
          autonomousCommand = new AutoDoNothing();
          break;
        case RightCargoShip:
          autonomousCommand = new AutoDoNothing();
          //new Hab1ToCargoShipEndRightSide();
          break;
        case LeftBottomRocket:
          autonomousCommand = new AutoDoNothing();
          break;
        case RightBottomRocket:
          autonomousCommand = new AutoDoNothing();
          break;
        case DoNothing:
          autonomousCommand = new AutoDoNothing();
          break;
        case DriveOffHab1:
          autonomousCommand = new PDriveToDistance(0.4, 4);
          break;
        case DriveOffHab2:
          autonomousCommand = new PDriveToDistance(0.4, 9);
          break;
        case TestMotionProfile:
          autonomousCommand = new AutoDoNothing();//FollowPath(PathManager.getInstance().profiles.get(3));
          break;
        default:
          autonomousCommand = new AutoDoNothing();
      }
    }
    //autonomousCommand.start();

    //Scheduler.getInstance().add(new Hab1ToCargoRightRocketLow());
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    //oi.reconfigureButtons();
  }

  @Override
  public void teleopInit() {

    //elevator.resetElevatorEncoder();

    // Init hatch target finding vision camera
    //cameraControlStateMachine.identifyTargets();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    System.out.println("---------------------");

    arm.SetIdleBrakeMode();
    Scheduler.getInstance().add(new LockArm());

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // Start your engines
    // defaultDriveTrain.start();
  }

  /**
   * This function is called periodically during operator control.
   */

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    //elevator.ZeroElevator();
  }

  @Override
  public void testPeriodic() {
  }

  public static double getDeltaTime() { return deltaTime; }

  public void updateSmartDashboard() {
    //liftGear.updateSmartDashboard();
    //driveTrain.updateSmartDashboard();
    arm.updateSmartDashboard();
    elevator.updateSmartDashboard();
    //frontLineDetector.updateSmartDashboard();
    //frontInfraredRangeFinder.updateSmartDashboard();
    SmartDashboard.putNumber("Delta Time", deltaTime);
    //SmartDashboard.putBoolean("Paths Loaded", PathManager.getInstance().isLoaded());
  }

  public void updateSmartDashboardRequired() {
    elevator.updateSmartDashboard();
    SmartDashboard.putNumber("Delta Time", deltaTime);
  }

  public enum AutonomousOptions {
    LeftCargoShip, RightCargoShip, LeftBottomRocket,
    RightBottomRocket, DoNothing, DriveOffHab1,
    DriveOffHab2, TestMotionProfile
  }
}
