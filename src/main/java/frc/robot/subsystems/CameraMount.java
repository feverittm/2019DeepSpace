/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import org.team997coders.spartanlib.hardware.roborio.Servo;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ControlCamera;

/**
 * A subsystem to define an automated camera mount that uses servos for panning and tilting.
 */
public class CameraMount extends org.team997coders.spartanlib.subsystems.CameraMount {
  private final double maxDegreesPerHeartbeat;
  private final CANifier canifier;
  private final LEDChannel lightRingLEDChannel;

  /**
   * Convenience constructor to hard-wire CameraMount to Roborio servo implementation.
   */
  public CameraMount(int tiltLowerLimitInDegrees, 
      int tiltUpperLimitInDegrees, 
      int panLowerLimitInDegrees,
      int panUpperLimitInDegrees,
      double slewRate180DegreesInSec,
      double heartbeatRateInMs,
      LEDChannel lightRingLEDChannel) {
    super(new Servo(RobotMap.Ports.panservo), 
      new Servo(RobotMap.Ports.tiltservo, 544, 2250),
      tiltLowerLimitInDegrees,
      tiltUpperLimitInDegrees,
      panLowerLimitInDegrees,
      panUpperLimitInDegrees);
      canifier = Robot.elevatorCanifier;
      this.lightRingLEDChannel = lightRingLEDChannel;
      this.maxDegreesPerHeartbeat = (180D / slewRate180DegreesInSec) / (1000D / heartbeatRateInMs);
    }

  /**
   * Set the default command to ControlCamera
   */
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ControlCamera());
  } 

  /**
   * Center the camera 90 degrees on both axes.
   */
  public void center() {
    // TODO: Pull this function into SpartanLib
    panToAngle(90d);
    tiltToAngle(90d);
  }

  /**
   * Set the output of the camera mount light ring.
   * @param percentOutput   PWM dutycycle expressed as a percentage.
   */
  public void setLightRingOutput(double percentOutput) {
    canifier.setLEDOutput(percentOutput, lightRingLEDChannel);
  }

  /**
   * Turn off the light ring.
   */
  public void setLightRingOff() {
    setLightRingOutput(0);
  }

  /**
   * Slew the camera.
   * 
   * @param panRate   A value between -1 and 1 the represents the percentage of maximum pan slewage.
   * @param tiltRate  A value between -1 and 1 the represents the percentage of maximum tilt slewage.
   */
  public void slew(double panRate, double tiltRate) {
    // TODO: Pulls this function into SpartanLib; remove SlewCamera command?
    double panAngle = getPanAngleInDegrees() + (maxDegreesPerHeartbeat * panRate);
    double tiltAngle = getTiltAngleInDegrees() + (maxDegreesPerHeartbeat * tiltRate);
    panToAngle(panAngle);
    tiltToAngle(tiltAngle);
  }

  /**
   * Updates the SmartDashboard with subsystem data
   */
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Camera Pan Angle", getPanAngleInDegrees());
    SmartDashboard.putNumber("Camera Tilt Angle", getTiltAngleInDegrees());
  }
}
