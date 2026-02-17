// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);// Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ShooterConstants // Keep in mind these values affect 2 motors working together
  {
    // Target Shooter Velocity in RPM - May be removed later if using a function to calculate distance then velocity of the ball
    public static final double SHOOTER_TARGET_VELOCITY_RPM = 3000.0;

    // Shooter PID Constants
    public static final double kP = 0.0001;// Needs Tuning - Proportional Gain
    public static final double kI = 0.0;
    public static final double kD = 0.0;// Needs Tuning - Derivative Gain
    // public static final double kIz = 0.0; --- IGNORE ---

    // Feed Forward Constants
    public static final double kS = 0.0; // Needs Tuning - Static Friction Gain. Needed to overcome initial static friction.
    public static final double kV = 0.0; // Needs Tuning - Velocity Gain. Relates voltage to velocity.
    public static final double kA = 0.0; // Needs Tuning - Acceleration Gain. Relates voltage to acceleration.

    // Shooter Motor Current Limit and Voltage
    public static final int SHOOTER_MOTORS_CURRENT_LIMIT = 40; // Amps
    public static final double SHOOTER_MOTORS_VOLTAGE = 12.0; // Volts

    public static final int LEADERSHOOTERID = 0; // Can ID constant for the lead shoter motor
    public static final int FOLLOWERSHOOTERID = 0; // Can ID constant for the follower shoter motor
    
    // CAN Timeout is milliseconds between motor controller communication with the rio. Frees up space in the command scheduler and on the CAN bus.
    public static final int CANTimeoutMs = 250; // milliseconds
  }

  public static class IntakeConstants {

    public static final double kRollerS = 0;
    public static final double kRollerV = 0;
    public static final double kRollerA = 0;


    public static final double kRollerP = 0.01;
    public static final double kRollerI = 0;
    public static final double kRollerD = 0;



    public static final double kArmS = 0;
    public static final double kArmV = 0;
    public static final double kArmA = 0;


    public static final double kArmP = 0.01;
    public static final double kArmI = 0;
    public static final double kArmD = 0;

    public static final int INTAKE_ROLLER_MOTORS_CURRENT_LIMIT = 40; // Amps
    public static final double INTAKE_ROLLER_MOTORS_VOLTAGE = 12.0; // Volts


    public static final int INTAKE_ARM_MOTORS_CURRENT_LIMIT = 40; // Amps
    public static final double INTAKE_ARM_MOTORS_VOLTAGE = 12.0; // Volts

    public static final int ARMID = 0;
    public static final int ROLLERID = 0;

  }

  public static class TransportConstants {

    public static final double kTransportS = 0;
    public static final double kTransportV = 0;
    public static final double kTransportA = 0;


    public static final double kTransportP = 0.01;
    public static final double kTransportI = 0;
    public static final double kTransportD = 0;

    public static final int Transport_MOTORS_CURRENT_LIMIT = 40; // Amps
    public static final double Transport_MOTORS_VOLTAGE = 12.0; // Volts

    public static final int TRANSPORTID = 0;

  } 

  public static class SpindexerConstants {

    public static final double kSpindexS = 0;
    public static final double kSpindexV = 0;
    public static final double kSpindexA = 0;


    public static final double kSpindexP = 0.01;
    public static final double kSpindexI = 0;
    public static final double kSpindexD = 0;

    public static final int SPINDEX_MOTORS_CURRENT_LIMIT = 40; // Amps
    public static final double SPINDEX_MOTORS_VOLTAGE = 12.0; // Volts

    public static final int SPINDEXERID = 0;
  }

  public static class TowerConstants {

    public static final double kCLIMB_P = 0;
    public static final double kCLIMB_I = 0;
    public static final double kCLIMB_D = 0;

    public static final int LEADERCLIMBID = 0;
    public static final int FOLLOWERCLIMBID = 0;

  }

  public static class driveToPoseConstants {
    public static final Translation2d BlueLeftTranslation = new Translation2d(0,0);
    public static final Translation2d BlueCenterTranslation = new Translation2d(0,0);
    public static final Translation2d BlueRightTranslation = new Translation2d(0,0);

    public static final Rotation2d BlueLeftRotation = new Rotation2d(0);
    public static final Rotation2d BlueCenterRotation = new Rotation2d(0);
    public static final Rotation2d BlueRightRotation = new Rotation2d(0);

    public static final Translation2d RedLeftTranslation = new Translation2d(0,0);
    public static final Translation2d RedCenterTranslation = new Translation2d(0,0);
    public static final Translation2d RedRightTranslation = new Translation2d(0,0);

    public static final Rotation2d RedLeftRotation = new Rotation2d(0);
    public static final Rotation2d RedCenterRotation = new Rotation2d(0);
    public static final Rotation2d RedRightRotation = new Rotation2d(0);
    
  }
}
