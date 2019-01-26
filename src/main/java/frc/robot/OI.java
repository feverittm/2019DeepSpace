/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.FollowLine;

public class OI {

  private Joystick gamepad1, gamepad2;

  private JoystickButton followLine;

  public OI() {
    gamepad1 = new Joystick(RobotMap.Ports.gamepad1);
    gamepad2 = new Joystick(RobotMap.Ports.gamepad2);

    followLine = new JoystickButton(gamepad1, 1);
    followLine.whenPressed(new FollowLine(1000));
  }

  public double getLeftYAxis() {
    return -gamepad1.getRawAxis(RobotMap.Ports.leftYAxis);
  }

  public double getRightXAxis() {
    return gamepad1.getRawAxis(RobotMap.Ports.rightXAxis);
  }

  public double getRightYAxis() {
    return -gamepad1.getRawAxis(RobotMap.Ports.rightYAxis);
  }

  public double deadBand(double value, double min, double max) {
    if (value > max) {
      return max;
    } else if (value < min) {
      return min;
    }
    
    return value;
  }

  // KEEP THESE COMMENTS
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
}