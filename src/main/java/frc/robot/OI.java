package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.vision.*;
import frc.robot.misc.Utils;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  Joystick driver;
  Joystick operator;

  // Commands associated with operator's control:
  private JoystickButton toggleHatch; // B 2
  public JoystickButton elevatorGoDown; // X 2
  public JoystickButton elevatorGoUp; // Y 2
  public JoystickButton ballIntake; // Right Bumper 2
  public JoystickButton ballOutake; // Left Bumper 2
  public JoystickButton flip;
  public JoystickButton ArmForward; // Back 2
  public JoystickButton ArmReverse; // Start 2

  // Commands associated with the driver's control
  public JoystickButton toggleLight;  // A 1
  private JoystickButton deployLandingGear; // B 1
  private JoystickButton deployBackLandingGear; // Y 1
  private JoystickButton retractLandingGear; // Back 1
  private JoystickButton limelightDrive;

  public int secretModeCounterA = 0, secretModeCounterB = 0;

  public OI() {
    // driver controls... game sticks control the motion of the robot
    // left stick Y-axis is drive power
    // right stick X-axis is drive direction
    driver = new Joystick(RobotMap.Buttons.GamePad1);
    operator = new Joystick(RobotMap.Buttons.GamePad2);

    // #region Driver's Controls
    deployLandingGear = new JoystickButton(driver, RobotMap.Buttons.buttonB);
    deployLandingGear.whenPressed(new ToggleFrontLandingGear());

    deployBackLandingGear = new JoystickButton(driver, RobotMap.Buttons.buttonY);
    deployBackLandingGear.whenPressed(new ToggleRearLandingGear());

    retractLandingGear = new JoystickButton(driver, RobotMap.Buttons.buttonBack);
    retractLandingGear.whenPressed(new RetractLandingGear());

    // flip = new JoystickButton(driver, RobotMap.Buttons.buttonX);
    // flip.whenPressed(new FlipArmChain());

    toggleLight = new JoystickButton(driver, RobotMap.Buttons.buttonA);
    toggleLight.whenPressed(new ToggleLight());

    limelightDrive = new JoystickButton(driver, RobotMap.Buttons.buttonStart);
    limelightDrive.whenPressed(new ApproachTarget(0.2, 19));

    // #endregion

    // #region Gamepad2 Controls
    elevatorGoUp = new JoystickButton(operator, RobotMap.Buttons.buttonY);
    elevatorGoUp.whileHeld(new ElevatorUppity());

    elevatorGoDown = new JoystickButton(operator, RobotMap.Buttons.buttonX);
    elevatorGoDown.whileHeld(new ElevatorDownity());

    //ArmForward = new JoystickButton(operator, RobotMap.Buttons.buttonStart);
    //ArmForward.whileHeld(new MoveArm(-0.2));

    //ArmReverse = new JoystickButton(operator, RobotMap.Buttons.buttonBack);
    //ArmReverse.whileHeld(new MoveArm(0.2));

    ballIntake = new JoystickButton(operator, RobotMap.Buttons.buttonRightShoulder);
    ballIntake.whileHeld(new BallIntake());

    ballOutake = new JoystickButton(operator, RobotMap.Buttons.buttonLeftShoulder);
    ballOutake.whileHeld(new BallOuttake());

    toggleHatch = new JoystickButton(operator, RobotMap.Buttons.buttonB);
    toggleHatch.whenPressed(new ToggleHatch());

  }

  // #endregion

  // #region Controller Data

  public double getLeftYAxis() {
    return Utils.condition_gamepad_axis(0.05, -driver.getRawAxis(RobotMap.Buttons.leftYAxis), -1.0, 1.0);
  }

  public double getRightXAxis() {
    return Utils.condition_gamepad_axis(0.05, driver.getRawAxis(RobotMap.Buttons.rightXAxis), -1, 1);
  }

  public double getRightYAxis() {
    return Utils.condition_gamepad_axis(0.05, -driver.getRawAxis(RobotMap.Buttons.rightYAxis), -1, 1);
  }

  // #endregion
}
