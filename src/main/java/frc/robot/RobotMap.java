/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static class LimeLight {
    public static String
      ledMode = "ledMode",
      cameraMode = "camMode",
      captureMode = "snapshot",
      streamSelection = "stream",
      targetX = "tx",
      targetY = "ty",
      targetArea = "ta",
      targetVisible = "tv";

    public static double
      reachedPanelArea = 22.2;
  }

  public static class Buttons {
    public static class Logitech {
      public static int 
        buttonA = 2,
        buttonB = 3,
        buttonX = 1,
        buttonY = 4,
        buttonLeftThumbstick = 11,
        buttonRightThumbstick = 12,
        leftJoystickHatAngle = 270,
        centerJoystickHatAngle = 0,
        rightJoystickHatAngle = 90,
        leftXAxis = 0,
        leftYAxis = 1;
      }

    public static int

      GamePad1 = 0,
      GamePad2 = 1, 
      buttonA = 1,
      buttonB = 2,
      buttonX = 3,            
      buttonY = 4,
      buttonLeftShoulder = 5,
      buttonRightShoulder = 6,
      buttonLeftThumbstick = 9,
      buttonRightThumbstick = 10,
      buttonBack = 7,
      buttonStart = 8,

      //AXIS PORTS
      leftXAxis = 0,
      leftYAxis = 1,
      rightXAxis = 4,
      rightYAxis = 5,
      buttonRightTrigger = 3,
      buttonLeftTrigger = 2,

      end_of_buttons;
  }

  public static class Ports {
    public static final SerialPort.Port AHRS = SerialPort.Port.kUSB;
    public static int
      //TALON PORTS
      leftTalon = 4,
      rightTalon = 1,

      //VICTOR PORTS
      leftVictor1 = 5,
      leftVictor2 = 6,
      rightVictor1 = 2,
      rightVictor2 = 3, 

      //BALL PORTS
      ballMotor = 12,

      //HATCH PORTS
      hatchSolenoid = 1,
  
      //ARM PORTS
      armSpark = 62,
      armCanifier = 23,

      // ELEVATOR PORTS
      elevatorCanifier = 29,
      masterElevatorMotor= 42,
      followerElevatorMotor = 43,

      //LANDING GEAR PORTS
      landingGearPiston = 0,
      landingGearFloorSensor = 1,
      rearGearPiston = 3,
      rearGearFloorSensor = 0,

      // placeholder so we can always just end with commas :-)
      end_of_ports = 999;
  }

  public static class Values {
    public static int 
      ticksPerRev = 4096; // Protobot
      //PDriveToDistance PID Variables
    public static double

      limeTestP = 0.015,
      limeTestI = 0.0,
      limeTestD = 0.0,
      limeTestF = 0.0,

      driveDistanceP = .003, //placeholders
      driveDistanceI = 0.0,
      driveDistanceD = 0.0,
      protobotTickPerFoot= 2449,

      // PDriveToAngle PID Variables
      driveAngleP = .007, //placeholders
      driveAngleI = 0.0,
      driveAngleD = 0.0,

      wheelDiameter = 6.0,
      inchesPerTick = (wheelDiameter*Math.PI)/4096, //inches per encoder tick
      ticksPerFoot = ((49152/(wheelDiameter*Math.PI)))*.9, //3940, //encoder ticks per foot

      // Arm and Elevator Values
      elevatorHeightAbsTolerance = 100,
      elevatorPidP = 0, //0.0006, 
      elevatorPidI = 0, //0.0000000001,
      elevatorPidD = 0, //0.006,
      elevatorPidF = 0, //0.0000,
      elevatorPidFMax = 0,
      elevatorTopHeight = 100000000, //placeholder

      armAngleAbsTolerance = 50,
      armSwitchHeight= 22775, //placeholer for height in order for arm to switch
      armBackParallel = 788,
      armFrontParallel = 288,
      armFrontLower = 288,
      armVertical = 501,

      armPidP = 0.0010, //0.0006
      armPidI = 0,
      armPidD = 0.001,
      armPidK = 0,
      armMaxPidF = 0.002, // 0.0005
      ticksToRadiansArm= 3.141592653589793238/(Math.abs(armBackParallel-armFrontParallel)),

      // Camera values
      leftAngleInDegrees = 75,
      rightAngleInDegrees = 115,
      
      // Elevator Deccel/Accel Values
      bottomElevatorAccelPosLimit = 5000,
      bottomElevatorLimitVelocity = -0.28,
      topElevatorAccelPosLimit = 49000,
      topElevatorLimitVelocity = 0.4,

      internalFlipTickCount = 118428, // Temp Number

      robotLength = 0.75, //in inches (includes bumpers)
      robotWidth = 0.8,	
      robotWheelBase = 0.62, // inches or 2.5ft or 0.6 meters.  Use 0.0254 meters/in or 39.37in/m
      robotWheelDia = 0.15;// Javadocs requests that this is in meters not feet-> 6/12; // remember all pf variables are in ft.  Need to convert when used.
  }

  public static class ElevatorHeights {
    //in inches
    public static int
      elevatorMiddleHeight = 28360,

      elevatorBackTopHatchHeight = 0, //impossible
      elevatorBackMiddleHatchHeight = 0, //impossible
      elevatorBackBottomHatchHeight = 0, //impossible
      elevatorBackShipHatchHeight = 8900, //impossible

      elevatorBackTopCargoHeight = 52400, // probably higher
      elevatorBackMiddleCargoHeight = 34400,
      elevatorBackBottomCargoHeight = 9100,
      elevatorBackShipCargoHeight = 24650,

      elevatorFrontTopHatchHeight = 52317, // probably higher
      elevatorFrontMiddleHatchHeight = 28360,
      elevatorFrontBottomHatchHeight = 0,
      elevatorFrontShipHatchHeight = 1520,

      elevatorFrontTopCargoHeight = 52400,
      elevatorFrontMiddleCargoHeight = 45360,
      elevatorFrontBottomCargoHeight = 22200,
      elevatorFrontShipCargoHeight = 38125,

      elevatorCollectCargoHeight = 0;

      public static int[]
      elevatorFrontHatchHeightArray = {
        elevatorFrontTopHatchHeight, 
        elevatorFrontMiddleHatchHeight, 
        elevatorFrontBottomHatchHeight 
      },
    
      elevatorBackHatchHeightArray = {
        elevatorBackTopHatchHeight, 
        elevatorBackMiddleHatchHeight,
        elevatorBackBottomHatchHeight
      },

      elevatorFrontCargoHeightArray = {
        elevatorFrontTopCargoHeight,
        elevatorFrontMiddleCargoHeight,
        elevatorFrontBottomCargoHeight
      },

      elevatorBackCargoHeightArray = {
        elevatorBackTopCargoHeight,
        elevatorBackMiddleCargoHeight,
        elevatorBackBottomCargoHeight
      };
  }
}
