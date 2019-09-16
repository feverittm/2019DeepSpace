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
import frc.robot.Robot;
import frc.robot.RobotMap;
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

  private double rampAccel = 0.5; // Use this value to see if the elevator is actually being deccelerated

  private CANDigitalInput limitSwitchTop;
  private CANDigitalInput limitSwitchBottom;
  private CANifier elevatorCanifier;
  // public int index = 0;
  // public double[] heightList;
  public boolean gamePieceType;
  // This is to switch between balls and hatches for elevator heights.
  //// Balls = true Hatches = false
  public boolean isZeroed;

  private double kP_simulation = RobotMap.Values.elevatorPidP;
  private double kI_simulation = RobotMap.Values.elevatorPidI;
  private double kD_simulation = RobotMap.Values.elevatorPidD;

  public Elevator() {
    super("Elevator", 0, 0, 0);
    if (Robot.isSimulation()) { // Check for simulation and update PID values
      pidController.setPID(kP_simulation, kI_simulation, kD_simulation);
    }
    pidController.setAbsoluteTolerance(0.005);

    master = new CANSparkMax(RobotMap.Ports.masterElevatorMotor, MotorType.kBrushless);
    follower = new CANSparkMax(RobotMap.Ports.followerElevatorMotor, MotorType.kBrushless);

    master.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    encoder = master.getEncoder();
    encoder.setPosition(0);
    encoder.setPositionConversionFactor(42);

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

    pidController.setOutputRange(-0.3, 0.5);

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
    SetPosition(GetPosition());

  }

  public double returnPIDInput() {
    return (GetPosition());
  }

  public void usePIDOutput(double value) {
    SetPower(value);
  }

  public void SetPosition(double height) {
    // System.out.println("Set elevator to go to height " + height);
    pidController.setSetpoint(height);
    // updateF();
  }

  public void resetElevatorEncoder() {
    elevatorCanifier.setQuadraturePosition(0, 10);
  }

  public int GetPosition() {
    return elevatorCanifier.getQuadraturePosition();
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

  public void Stop() {
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
   * This function processes the power given and limits it based on position and
   * current speed to determine an appropriate speed to go at for a smooth, nice
   * elevator. It probably doesn't work... PLEASE ADJUST THE ACCELERATION INSTEAD
   * OF DELETING / NOT USING THIS
   * 
   * @param pow The desired power which you shall not receive
   * 
   *            Timothy: Our cpu usage is hella high. Hunter: Let me run this
   *            large processing function to determine the speed we should go at.
   *            Timothy: but the RIO is gonna die... Hunter: ... i commented
   *            it.... Timothy: Bad Hunter. Hunter: It's fine I'll just disable
   *            the logger. [Tests robot] Hunter and Timothy: .... Hunter: It's
   *            doing some weird stuff... Timothy: If only we had the logger...
   *            >:C
   */
  public void setDeccelPower(double pow) {
    double last = master.get(); // The last set power to the motor
    double hek = pow;
    boolean didMod = false; // Did I need to alter the power

    double deltaTime = Robot.getDeltaTime();

    if (Math.abs(pow) > Math.abs(last) + (rampAccel * deltaTime)) { // Did is the motor going to over accelerate?
      hek = last + ((last / Math.abs(last)) * (rampAccel * deltaTime)); // Limit how much it changes
      didMod = true; // Record it
    }

    if (didMod) { // Did you mod it?
      if (hek < 0) { // Is the new values moving up
        if ((GetPosition() < RobotMap.Values.bottomElevatorAccelPosLimit)
            && (hek < RobotMap.Values.bottomElevatorLimitVelocity)) { // Is it approching the bottom of the elevator and
                                                                      // is going rather fast?
          hek = RobotMap.Values.bottomElevatorLimitVelocity; // Limit the velocity even more
        }
      } else if ((GetPosition() > RobotMap.Values.topElevatorAccelPosLimit)
          && (hek > RobotMap.Values.topElevatorLimitVelocity)) { // Is it approching the top of the elevator and is
                                                                 // going rather fast?
        hek = RobotMap.Values.topElevatorLimitVelocity; // Limit the velocity even more
      }
    }

    SetPower(hek); // Apply new velocity
  }

  public double CtreToSparkEncoder(double ctre) {
    return ((ctre / 1024) / 2.5) * 42;
  }

  public void updateF() {
    // the midpoint of the elevator pulls the constant force spring
    // and causes a larger downward force.
    if (GetPosition() > RobotMap.ElevatorHeights.elevatorMiddleHeight) {
      pidController.setF(RobotMap.Values.elevatorPidFMax);
    } else {
      pidController.setF(RobotMap.Values.elevatorPidF);
    }
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
    // setDefaultCommand(new LockElevator());
  }

  public void getElevatorData(ElevatorData e) {
    e.output = master.getAppliedOutput();
    e.current = master.getOutputCurrent();
    e.position = GetPosition();
    e.velocity = elevatorCanifier.getQuadratureVelocity();
    e.bottom = GetBottomLimitSwitch();
    e.top = limitSwitchTop.get();
    e.position = GetPosition();
    e.setpoint = pidController.getSetpoint();
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
