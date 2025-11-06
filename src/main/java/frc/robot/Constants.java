// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  // if only vscode had color coded comments...
  // !nevermind better comments is a extension I can use!

  public static class IntakeConstants {
    // *Intake Subsystem Ports
    public static final int intakeCanID = 9;
    public static final int drawBridgeCanID = 1;

    // *Rollers
    // Speeds
    public static final double intakeRunSpeed = 1;
    public static double intakePurgeSpeed = 1;

    // *DrawBridge
    // PID vars
    // TODO tune pid vars
    public static final double DBkP = 2;
    public static final double DBkI = 0;
    public static final double DBkD = 1;
    // TODO find minimal vars
    public static final double DRAWBRIDGE_ALLOWABLE_ERROR = 5;
    public static final double DRAWBRIDGE_MAXINTEGRAL = 4000;
    public static final double DRAWBRIDGE_INTEGRAL_ZONE = 100;

    // speeds
    public static double loweredIntakeValue = 15500;
    public static double raisedIntakeValue = 250;
    // config
    public static boolean drawBridgeInverted = true;
  }

  public static class IndexerConstants {
    // *Indexer Subsystem Ports
    public static final int indexerCanID = 10;
    public static final int frontSensorChannel = 0;
    public static final int backSensorChannel = 4;
    public static final double IkP = 0;
    public static final double IkI = 0;
    public static final double IkD = 0;
    // *Indexer
    // speeds
    public static double indexerRunSpeed = 1;
    public static double indexerPurgeSpeed = -1;
    public static double indexerShootSpeed = 1;
    public static boolean indexerInverted = false;
  }

  public static class ShooterConstants {

    public static final int shooterCanID = 2;
    public static final int shooterCurrentLimit = 40;
    public static final double shooterkP = 0.08;
    public static final double shooterkI = 0;
    public static final double shooterkD = 0;
    // TODO placeholder values
    public static final double shooterkS = 0;
    public static final double ShooterStableRPMTime = 0;
    public static int shooterRPMTolerance = 10;

  }
}