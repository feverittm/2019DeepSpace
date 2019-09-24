/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.data.ElevatorData;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.LockElevator;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.CANifier;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Add your docs here.
 */
public class Elevator extends PIDSubsystem {
  private CANSparkMax master, follower;
  private CANEncoder encoder;
  private PIDController pidController;

  private CANDigitalInput limitSwitchTop;
  private CANDigitalInput limitSwitchBottom;
  private CANifier elevatorCanifier;

  public boolean isZeroed;

  public Elevator() {
    super("Elevator", 0, 0, 0);
    pidController.setAbsoluteTolerance(0.005);

    master = new CANSparkMax(RobotMap.Ports.masterElevatorMotor, MotorType.kBrushless);
    follower = new CANSparkMax(RobotMap.Ports.followerElevatorMotor, MotorType.kBrushless);

    master.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    encoder = master.getEncoder();
    //encoder.setPosition(0);
    //encoder.setPositionConversionFactor(42);

    // This line must be this way now that the canifiers are shared recources
    elevatorCanifier = new CANifier(RobotMap.Ports.elevatorCanifier);
    limitSwitchTop = new CANDigitalInput(master, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);
    limitSwitchTop.enableLimitSwitch(true);

    limitSwitchBottom = new CANDigitalInput(master, LimitSwitch.kForward, LimitSwitchPolarity.kNormallyOpen);
    limitSwitchBottom.enableLimitSwitch(true);

    master.setIdleMode(IdleMode.kBrake);
    follower.setIdleMode(IdleMode.kBrake);

    master.setInverted(true);
    follower.setInverted(true);

    follower.follow(master, true); // reverse the follower in the follow command

    pidController.setOutputRange(-0.5, 0.5);

    // until we fix the bottom limit switch!
    // isZeroed = limitSwitchBottom.get();

    // update the PID controls on the smart dashboard.
    SmartDashboard.putNumber("Elevator/Elevator Pid P", RobotMap.Values.elevatorPidP);
    SmartDashboard.putNumber("Elevator/Elevator Pid I", RobotMap.Values.elevatorPidI);
    SmartDashboard.putNumber("Elevator/Elevator Pid D", RobotMap.Values.elevatorPidD);
    SmartDashboard.putNumber("Elevator/Elevator Pid F", RobotMap.Values.elevatorPidF);
    pidController.setPID(RobotMap.Values.elevatorPidP, RobotMap.Values.elevatorPidI, RobotMap.Values.elevatorPidD,
        RobotMap.Values.elevatorPidF);

    resetElevatorEncoder();
    isZeroed = true;

    // last thing... lock elevator at the current position (the bottom for now) for the start.
    //SetPosition(GetPosition());

  }

  public double returnPIDInput() {
    return GetPosition();
  }

  public void usePIDOutput(double value) {
    SetPower(value);
  }

  public void resetElevatorEncoder() {
    stop();
    elevatorCanifier.setQuadraturePosition(0, 10);
  }

  public int GetPosition() {
    return elevatorCanifier.getQuadraturePosition();
  }

  public void SetPosition(double height) {
    // System.out.println("Set elevator to go to height " + height);
    pidController.setSetpoint(height);
    // updateF();
  }

  public double getInternalEncoderPos() {
    return encoder.getPosition();
  }

  public boolean GetBottomLimitSwitch() {
    return limitSwitchBottom.get();
  }

  public boolean getTopLimitSwitch() {
    return limitSwitchTop.get();
  }

  public double getMasterTemp() {
    return master.getMotorTemperature();
  }

  public double getFollowerTemp() {
    return follower.getMotorTemperature();
  }

  public void stop() {
    pidController.disable();
    master.set(0);
  }

  public void SetPower(double volts) {
    master.set(volts);
  }

  public boolean atSetpoint() {
    return pidController.onTarget();
  }

  /**
   * Reset the poistion of the elevator encoder. If the limit switches are
   * available then trigger reset using them. Otherwise assume that the initial
   * position of the elevator is at the bottom and allow a reset at that point.
   */
  public void ZeroElevator() {
    if (limitSwitchBottom.get()) {
      resetElevatorEncoder();
      isZeroed = true;
      // Robot.arm.setArmFrontLimit(RobotMap.Values.armFrontLower);
    } else {
      // Robot.arm.setArmFrontLimit(RobotMap.Values.armFrontParallel);
    }
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new LockElevator());
  }

  public void getElevatorData() {
    ElevatorData.output = master.getAppliedOutput();
    ElevatorData.current = master.getOutputCurrent();
    ElevatorData.height = GetPosition();
    ElevatorData.velocity = elevatorCanifier.getQuadratureVelocity();
    ElevatorData.bottom = GetBottomLimitSwitch();
    ElevatorData.top = limitSwitchTop.get();
    ElevatorData.setpoint = pidController.getSetpoint();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Elevator/Elevator Output", master.getAppliedOutput());
    SmartDashboard.putNumber("Elevator/Elevator Height: ", GetPosition());
    SmartDashboard.putBoolean("Elevator/Bottom Limit Switch", limitSwitchBottom.get());
    SmartDashboard.putBoolean("Elevator/Top Limit Switch", limitSwitchTop.get());
    SmartDashboard.putNumber("Elevator/Elevator Pid F", pidController.getF());
    SmartDashboard.putNumber("Elevator/Setpoint", pidController.getSetpoint());
    // SmartDashboard.putNumber("Elevator/Elevator", master.getOutputCurrent());
    // SmartDashboard.putNumber("Elevator/Elevator Internal Encoder",
    // getInternalEncoderPos());
    // SmartDashboard.putNumber("Elevator/Elevator Master Temp", getMasterTemp());
    // SmartDashboard.putNumber("Elevator/Elevator Follower Temp",
    // getFollowerTemp());

  }
}
