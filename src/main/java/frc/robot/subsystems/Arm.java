package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.data.ArmData;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Aether's Arm. One SparkMax/NEO to move the arm, and a canifier
 * connected absolute PWM encoder to measure the arm angle.
 */
public class Arm extends PIDSubsystem {

  public double setpoint = 0;
  private CANSparkMax sparkMax;
  private CANEncoder internalEncoder;
  private CANifier armCanifier;
  private PIDController pidController;

  private final double MAX = 1022;
  private final double LIMIT = 500;
  private double prevRead = -1;
  private int revs = 0;
  private int flipModifier = 1;
  private double fConstant = 0.002;
  public boolean armState;

  public Arm() {
    super("Arm", 0, 0, 0);
    setAbsoluteTolerance(0.05);
    pidController.setContinuous(false);
    pidController.disable();

    sparkMax = new CANSparkMax(RobotMap.Ports.armSpark, MotorType.kBrushless);
    sparkMax.restoreFactoryDefaults();
    sparkMax.setInverted(true);

    internalEncoder = sparkMax.getEncoder();
    double conversionFactor = RobotMap.Values.internalFlipTickCount
        / (RobotMap.Values.armBackParallel - RobotMap.Values.armFrontParallel);
    internalEncoder.setPositionConversionFactor(conversionFactor);

    armCanifier = new CANifier(RobotMap.Ports.armCanifier);

    // initialize arm encoder assuming that it is at zero position.
    // prevRead = getRawEncoder();
  }

  /**
   * Simple duty-cycle control of the motor speed.
   */
  public void setPower(double speed) {
    pidController.disable();
    sparkMax.set(speed);
  }

  /**
   * @return get the feedback sensor data for driving the PID controller.
   */
  protected double returnPIDInput() {
    return readEncoder();
  }

  /**
   * Apply the PID results to move the arm.
   */
  protected void usePIDOutput(double output) {
    sparkMax.set(output);
  }

  // This function only works if the inital read of the arm is horizontal
  public double UpdateF() {
    // pidController.setFF(0);
    double f = -1 * Math.cos(((readEncoder() - RobotMap.Values.armFrontParallel) * RobotMap.Values.ticksToRadiansArm))
        * fConstant;
    // pidController.setFF(f);
    return f;
  }

  /**
   * @return Get the output current from the NEO SparkMax.
   */
  public double getCurrent() {
    return sparkMax.getOutputCurrent();
  }

  /**
   * @return condition the raw encoder to handle installation errors.
   * Issue with this calculation is that it requires initialization (prevRead)
   */
  public double readEncoder() {
    double newVal = getRawEncoder();

    if (prevRead == -1) {
      prevRead = newVal;
    }

    if (Math.abs(prevRead - newVal) > LIMIT) {
      if (newVal > prevRead) {
        revs -= flipModifier;
      } else {
        revs += flipModifier;
      }
    }
    prevRead = newVal;
    return (double) ((revs * MAX) + newVal);
  }

  /**
   * @return Get the relative encoder from canifier connected CTRE mag encoder
   */
  public double getRawEncoder() {
    double[] a = new double[2];
    armCanifier.getPWMInput(PWMChannel.PWMChannel0, a);
    // SmartDashboard.putNumber("Duty Cycle", a[1]);
    return a[0];
  }

  /**
   * set the arm motor to brake mode. Short the motor leads to cause the Reverse
   * EMF to resist motion. How does this work with brushless motors with three
   * leads.
   */
  public void SetIdleBrakeMode() {
    sparkMax.setIdleMode(IdleMode.kBrake);
  }

  /**
   * set the arm motor to coast mode. In reality leave the motor leads open when
   * power is removed from the motor
   */
  public void SetIdleCoastMode() {
    sparkMax.setIdleMode(IdleMode.kCoast);
  }

  /**
   * switch off all arm power
   */
  public void stop() {
    // need to disable the PID and stop the motor
    pidController.disable();
    sparkMax.set(0.0);
  }

  /**
   * @return Get the Arm Neo temperature.
   */
  public double getMotorTemp() {
    return sparkMax.getMotorTemperature();
  }

  /**
   * Long term will be to make the arm Lock in position by default
   */
  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new LockArm());
  }

  /**
   * Update a local Sensor storage array
   */
  public void getArmData() {
    ArmData.output = sparkMax.getAppliedOutput();
    ArmData.current = sparkMax.getOutputCurrent();
    ArmData.setpoint = pidController.getSetpoint();
    ArmData.raw_encoder = getRawEncoder();
    ArmData.velocity = 0;
    ArmData.angle = readEncoder();
  }

  /**
   * Update the SmartDashboard for the Arm subsystems
   */
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Arm/Arm Absolute Raw", getRawEncoder());
    SmartDashboard.putNumber("Arm/Absolute Parsed", readEncoder());
    SmartDashboard.putNumber("Arm Angle",
        ((readEncoder() - RobotMap.Values.armFrontParallel) * RobotMap.Values.ticksToRadiansArm));
    SmartDashboard.putNumber("Arm/Arm F", pidController.getF());
    SmartDashboard.putNumber("Arm/Arm voltage", sparkMax.getAppliedOutput());
    SmartDashboard.putNumber("Arm/Arm Internal Read", internalEncoder.getPosition());
    SmartDashboard.putNumber("Arm/Arm Current", getCurrent());
    SmartDashboard.putNumber("Arm/Setpoint", pidController.getSetpoint());
  }

  /**
   * @return Are we at our setpoint? Determine if we are close enough to the
   *         setpoint
   */
  public boolean pidError() {
    double tolerance = RobotMap.Values.armAngleAbsTolerance;
    double setpoint = pidController.getSetpoint();

    return (readEncoder() > setpoint - tolerance) && (readEncoder() < setpoint + tolerance);
  }

}