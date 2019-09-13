package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.data.ArmData;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Arm extends PIDSubsystem {

  public double setpoint = 0;
  private CANSparkMax sparkMax;
  private CANEncoder internalEncoder;
  private CANifier dataBus;
  private PIDController pidController;

  // Read Encoder Vars
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

    sparkMax = new CANSparkMax(RobotMap.Ports.armSpark, MotorType.kBrushless);
    sparkMax.restoreFactoryDefaults();
    sparkMax.setInverted(true);

    internalEncoder = sparkMax.getEncoder();
    double conversionFactor = RobotMap.Values.internalFlipTickCount / (RobotMap.Values.armBackParallel - RobotMap.Values.armFrontParallel);
    internalEncoder.setPositionConversionFactor(conversionFactor);     
    dataBus = Robot.armCanifier;
  }

  public void setPower(double speed) {
    sparkMax.set(speed);
  }

  protected double returnPIDInput() {
    return readEncoder();
  }

  protected void usePIDOutput(double output) {
    setPower(output);
  }

  // This function only works if the inital read of the arm is horizontal
  public double UpdateF(){
    //pidController.setFF(0);
    double f = -1 * Math.cos(((readEncoder() - RobotMap.Values.armFrontParallel) * RobotMap.Values.ticksToRadiansArm)) * fConstant;
    //pidController.setFF(f);
    return f;
  }

  public double getCurrent() {
    return sparkMax.getOutputCurrent();
  }

  public double readEncoder() {
    double newVal = getRawEncoder();
    
    /*if (test) {
      newVal = testInput; // Read test input
    } else {
      newVal = getRawEncoder(); // Read encoder data
    }*/

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
    return (double)((revs * MAX) + newVal);
  }

  public double getRawEncoder() {
    double[] a = new double[2];
    dataBus.getPWMInput(PWMChannel.PWMChannel0, a);
    SmartDashboard.putNumber("Duty Cycle", a[1]);
    return a[0];
  }

  public void SetIdleBrakeMode() {
    sparkMax.setIdleMode(IdleMode.kBrake);
  }

  public void SetIdleCoastMode() {
    sparkMax.setIdleMode(IdleMode.kCoast);
  }

  public void stop() {
    // need to disable the PID and stop the motor
    sparkMax.set(0.0);
  }

  public double getMotorTemp() {
    return sparkMax.getMotorTemperature();
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new LockArm());
  }

  public ArmData getArmData() {
    ArmData a = new ArmData();
    a.output = sparkMax.getAppliedOutput();
    a.current = sparkMax.getOutputCurrent();
    a.ticks = readEncoder();
    a.velocity = 0;

    return a;
  }

  public void updatePID() {
    //pidController.setP(SmartDashboard.getNumber("Arm Pid P", RobotMap.Values.armPidP));
    //pidController.setI(SmartDashboard.getNumber("Arm Pid I", RobotMap.Values.armPidI));
    //pidController.setD(SmartDashboard.getNumber("Arm Pid D", RobotMap.Values.armPidD));
    //fConstant = SmartDashboard.getNumber("Arm Pid F", RobotMap.Values.armMaxPidF);
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Arm/Arm Absolute Raw", getRawEncoder());
    SmartDashboard.putNumber("Arm/Absolute Parsed", readEncoder());
    SmartDashboard.putNumber("Arm Angle", ((readEncoder() - RobotMap.Values.armFrontParallel) * RobotMap.Values.ticksToRadiansArm));
    //SmartDashboard.putBoolean("Arm/Disc Brake state: ", discBrake.get());
    //SmartDashboard.putBoolean("Arm/Arm forward limit switch", getForwardLimitSwitch());

    SmartDashboard.putNumber("Arm/Arm F", pidController.getF());
    //SmartDashboard.putBoolean("Arm/Arm Front Limit Switch", frontLimitSwitch.get());
    //SmartDashboard.putBoolean("Arm/Arm Back Limit Switch", backLimitSwitch.get());
    SmartDashboard.putNumber("Arm/Arm voltage", sparkMax.getAppliedOutput());
    SmartDashboard.putNumber("Arm/Arm Internal Read", internalEncoder.getPosition());
    //SmartDashboard.putNumber("Arm/Arm Current", getCurrent());
    //SmartDashboard.putNumber("Arm/Arm Motor Temp", getMotorTemp());
  }


//Started copying over Hunter's PID code from SetArmPosition cuz PID ain't
//supposed to go in commands.
  public boolean pidError(double setpoint, double tolerance) {
    return (readEncoder() > setpoint - tolerance) && (readEncoder() < setpoint + tolerance);
  }

}