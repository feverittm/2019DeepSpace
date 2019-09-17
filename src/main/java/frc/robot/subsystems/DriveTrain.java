package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDrive;
import frc.robot.misc.GearBox;
import frc.robot.misc.RoboMisc;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

/**
 * This is our drivetrain. This year I split up the configuration for the
 * drivetrain into another class so this one isn't comically long. We have a few
 * different methods such as normal set volts and stop volts. Along with
 * SetVoltsDecel and SetPosition. NOTE: Grayson hates deceleration code but he
 * reeeealllly needs it. Also not sure if the decel code works so definetly
 * check that first. Make sure the dampRate doesn't need adjusting.
 */
public class DriveTrain extends Subsystem {

  public boolean decell = true;

  public double maxSpeed = 0.5;

  // Decell Data
  private double prevY = 0;

  // GearBox class stores information for the motor controllers for one gearbox
  private final WPI_TalonSRX leftTalon, rightTalon;
  private final WPI_VictorSPX leftVictor1, leftVictor2, rightVictor1, rightVictor2;
  private AHRS gyro;

  private NetworkTable table;

  /**
   * A default constructor for the drivetrain to use from the robot program.
   */
  public DriveTrain() {
    // This uses the RoboMisc function standTalonSRXSetup(int, int, int, boolean) to
    // initialize a Talon and 2 slave victors
    this(
        RoboMisc.standTalonSRXSetup(RobotMap.Ports.leftTalon, RobotMap.Ports.leftVictor1, RobotMap.Ports.leftVictor2,
            false),
        RoboMisc.standTalonSRXSetup(RobotMap.Ports.rightTalon, RobotMap.Ports.rightVictor1, RobotMap.Ports.rightVictor2,
            true),
        null, NetworkTableInstance.getDefault().getTable("SmartDashboard"));
  }

  /**
   * Drivetrain constructor to use for testing purposes.
   * 
   * @param leftBox
   * @param rightBox
   * @param gyro
   * @param smartDashboardNetworkTable
   */
  public DriveTrain(GearBox leftBox, GearBox rightBox, AHRS gyro, NetworkTable smartDashboardNetworkTable) {

    // Grab the objects created by the RoboMisc function and store them in this
    // class
    leftTalon = leftBox.talon;
    rightTalon = rightBox.talon;
    leftVictor1 = leftBox.victor1;
    leftVictor2 = leftBox.victor2;
    rightVictor1 = rightBox.victor1;
    rightVictor2 = rightBox.victor2;

    if (gyro == null) {
      try {
        gyro = new AHRS(RobotMap.Ports.AHRS);
      } catch (RuntimeException e) {
        System.out.println("DT- The gyro broke.");
        gyro = null;
      }
    }

    resetGyro();
    resetEncoders();
    setCoast();

    table = smartDashboardNetworkTable;
  }

  /**
   * reset the gyro and the class variables this can be called any time.
   * 
   */
  public void resetGyro() {
    if (gyro != null) {
      gyro.reset();
      gyro.zeroYaw();
    } else {
      // programmer.sadness()
    }
  }

  /**
   * @return full angle of the robot (0-360)
   */
  public double getGyroAngle() {
    if (gyro != null) {
      return gyro.getFusedHeading();
    } else {
      return 0;
    }
  }

  /**
   * @return simple heading (-180<>+180) of robot.  Sometimes
   * this is more useful for driving corrections
   */
  public double getHeading() {
    if (gyro != null) {
      return (gyro.getYaw());
    } else {
      return 0.0;
    }
  }

  /**
   * Set a percent input to the left and right talons
   * 
   * @param left  Percentage input for the left talon.
   * @param right Percentage input for the right talon.
   */
  public void setVolts(double left, double right) {
    leftTalon.set(ControlMode.PercentOutput, left);
    rightTalon.set(ControlMode.PercentOutput, right);
  }

  /**
   * Stop the Robot!
   * Sets the percentage output for the drivetrain motor controllers
   * for the left and right side talons to zero
   */
  public void stopVolts() {
    // Set Motor Volts to 0
    leftTalon.set(ControlMode.PercentOutput, 0);
    rightTalon.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Acceleration mode drive.  Set-up a reasonable linear accel
   * profile for the drivetrain using a class supplied ramp
   * rate.
   * Note: prev_y is initialized in the constructor
   */
  public void setRampArcadeVolts(double m_ramp, double m_power, double m_turn) {
    double newY = m_power;

    // prevY = (leftTalon.getMotorOutputPercent() +
    // rightTalon.getMotorOutputPercent()) / 2;

    double maxIncrement = Robot.getDeltaTime() * m_ramp;

    if (Math.abs(m_power - prevY) > maxIncrement) {
      double sign = (m_power - prevY) / Math.abs(m_power - prevY);
      newY = (maxIncrement * sign) + prevY;
    }

    if (Math.abs(newY) > maxSpeed) {
      newY = (Math.abs(newY) / newY) * maxSpeed;
    }

    leftTalon.set(ControlMode.PercentOutput, newY + m_turn);
    rightTalon.set(ControlMode.PercentOutput, newY - m_turn);

    prevY = newY;
  }

  /**
   * Sets the desired tick position for the left and right talon
   * 
   * @param left  Desired tick position for the left talon
   * @param right Desired tick position for the right talon
   */
  public void setPosition(double left, double right) {
    leftTalon.set(ControlMode.Position, left);
    rightTalon.set(ControlMode.Position, right);
  }

  /**
   * Gets the left talon's position output
   * 
   * @return Current tick position of the left talon's encoder
   */
  public double leftEncoderTicks() {
    leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    return leftTalon.getSelectedSensorPosition(0);
  }

  /**
   * Gets the right talon's position output
   * 
   * @return Current tick position of the right talon's encoder
   */
  public double rightEncoderTicks() {
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    return rightTalon.getSelectedSensorPosition(0);
  }

  /**
   * Gets the left talon's velocity output
   * 
   * @return Current tick velocity of the left talon's encoder
   */
  public double leftEncoderVelocity() {
    leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    return leftTalon.getSelectedSensorVelocity(0);
  }

  /**
   * Gets the right talon's velocity output
   * 
   * @return Current tick velocity of the right talon's encoder
   */
  public double rightEncoderVelocity() {
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    return rightTalon.getSelectedSensorVelocity(0);
  }

  /**
   * Resets the encoders for both talons to 0
   */
  public void resetEncoders() {
    leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); /* PIDLoop=0,timeoutMs=0 */
    leftTalon.setSelectedSensorPosition(0, 0, 10);
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); /* PIDLoop=0,timeoutMs=0 */
    rightTalon.setSelectedSensorPosition(0, 0, 10);
  }

  /**
   * Sets the talons and their follow victors into BrakeMode
   */
  public void setBrake() {
    leftTalon.setNeutralMode(NeutralMode.Brake);
    rightTalon.setNeutralMode(NeutralMode.Brake);

    leftVictor1.setNeutralMode(NeutralMode.Brake);
    rightVictor1.setNeutralMode(NeutralMode.Brake);
    leftVictor2.setNeutralMode(NeutralMode.Brake);
    rightVictor2.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Sets the talons and their follow victors into CoastMode
   */
  public void setCoast() {
    leftTalon.setNeutralMode(NeutralMode.Coast);
    rightTalon.setNeutralMode(NeutralMode.Coast);

    leftVictor1.setNeutralMode(NeutralMode.Coast);
    rightVictor1.setNeutralMode(NeutralMode.Coast);
    leftVictor2.setNeutralMode(NeutralMode.Coast);
    rightVictor2.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Updates the SmartDashboard with subsystem data
   */
  public void updateSmartDashboard() {
    // Factor out direct calls to SmartDashboard and use passed in table
    // so that we can test.
    table.getEntry("Left Ticks DriveTrain").setDouble(leftEncoderTicks());
    table.getEntry("Right Ticks DriveTrain").setDouble(rightEncoderTicks());
    table.getEntry("Left Velocity Drivetrain").setDouble(leftEncoderVelocity());
    table.getEntry("Right Velocity Drivetrain").setDouble(rightEncoderVelocity());
    table.getEntry("Gyro angle").setDouble(getGyroAngle());

    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    SmartDashboard.putNumber("Prev Y", prevY);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArcadeDrive());
    // setDefaultCommand(new TankDrive());
  }
}
