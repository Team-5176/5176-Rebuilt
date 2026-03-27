// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
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
  public static final double MAX_SPEED  = Units.feetToMeters(19.5);// Maximum speed of the robot in meters per second, used to limit acceleration.

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
    public static double SHOOTER_TARGET_VELOCITY_RPM = 1800.0;//3000

    // shooter speeds: right(B): middle(Y): left(X):

    // Shooter PID Constants
    public static final double kP = 0.0008;// 0.0004 is a GOOD value for shooting! Needs Tuning - Proportional Gain
    public static final double kI = 0.0;
    public static final double kD = 0.0;// Needs Tuning - Derivative Gain
    // public static final double kIz = 0.0; --- IGNORE ---

    // Feed Forward Constants
    // public static final double kS = 0.20; // Needs Tuning - Static Friction Gain. Needed to overcome initial static friction.
    public static final double kV = 0.009; // Needs Tuning - Velocity Gain. Relates voltage to velocity.
    // public static final double kA = 1.0; // Needs Tuning - Acceleration Gain. Relates voltage to acceleration.

    // Shooter Motor Current Limit and Voltage
    public static final int SHOOTER_MOTORS_CURRENT_LIMIT = 60; // Amps
    public static final double SHOOTER_MOTORS_VOLTAGE = 10.0; // Volts

    public static final int LEADERSHOOTERID = 12; // Can ID constant for the lead shoter motor
    public static final int FOLLOWERSHOOTERID = 11; // Can ID constant for the follower shoter motor
  }

  public static class IntakeConstants {

    public static final double kRollerV = 0.19;
    public static final double kRollerA = .512;
    public static final double kRollerS = 0.1;

    public static final double kRollerP = 0.0008;
    public static final double kRollerI = 0;
    public static final double kRollerD = 0;

    public static final double INTAKE_ROLLER_VELOCITY_RPM = 800.0;

    public static final double kArmV = 0.91;
    public static final double kArmA = 0.042;
    public static final double kArmG = 0.60;

    public static final double kArmP = 0.05; //was 0.05 - had to up this value in an attempt to increase retracting speed.
    public static final double kArmI = 0;
    public static final double kArmD = 0;

    public static final double kArmRotations = -12;
    public static final double kArmRetractPos = 0; //was -0.2 

    public static final int INTAKE_ROLLER_MOTORS_CURRENT_LIMIT = 60; // Amps
    public static final double INTAKE_ROLLER_MOTORS_VOLTAGE = 10.0; // Volts


    public static final int INTAKE_ARM_MOTORS_CURRENT_LIMIT = 40; // Amps
    public static final double INTAKE_ARM_MOTORS_VOLTAGE = 10.0; // Volts
    public static final int ARMID = 32;
    public static final int ROLLERID1 = 33;


  }

  public static class TransportConstants {

    // public static final double kTransportS = 0.001;
    public static final double kTransportV = 0.01;
    // public static final double kTransportA = 1.0;

    public static final double TRANSPORT_VELOCITY_RPM = 1200.0; //actually this is 4000 rpm, probably PID?? I, Thomas PRESIDENT OF CLUB OF ROBOTICS ANNO DOMINI 2026 (andrew told me to add that), made this btw
    
    public static final double kTransportP = 0.00055; //0.0002
    public static final double kTransportI = 0;
    public static final double kTransportD = 0; //0.0008

    public static final int Transport_MOTORS_CURRENT_LIMIT = 40; // Amps
    public static final double Transport_MOTORS_VOLTAGE = 10.0; // Volts

    public static final int TRANSPORTID = 13;

  } 

  public static class SpindexerConstants {


    public static final double kSpindexV = 0.53;
    public static final double kSpindexA = 0.17;


    public static final double kSpindexP = 0.0013; //was 0.0019 - changed it lower since it's jittering a bit
    public static final double kSpindexI = 0;
    public static final double kSpindexD = 0;

    public static final int SPINDEX_MOTORS_CURRENT_LIMIT = 40; // Amps
    public static final double SPINDEX_MOTORS_VOLTAGE = 10.0; // Volts

    public static final int SPINDEXERID = 31;

    public static final double SPINDEXER_TARGET_VELOCITY_RPM = 4000;
  }

  public static class TowerConstants {

    public static final Pose2d TOWERALIGN = new Pose2d();
    public static final Translation2d TOWERTRANSLATION = new Translation2d(0,0);
    public static final Rotation2d TOWERROTATION = new Rotation2d(0);

    public static final double CLIMBPOS = 30.0;
    public static final double RESETPOS = 0;

    // public static final int FLIPPERS_MOTOR_CURRENT_LIMIT = 40;
    // public static final double FLIPPERS_MOTOR_VOLTAGE = 12;

    // public static final double kFlippersP = 0;
    // public static final double kFlippersI = 0;
    // public static final double kFlippersD = 0;

    // public static final double kFlippersS = 0;
    // public static final double kFlippersV = 0;
    // public static final double kFlippersA = 0;

    public static final double kCLIMB_P = 0.5;
    public static final double kCLIMB_I = 0;
    public static final double kCLIMB_D = 0;

    public static final double kCLIMB_S = 0;
    public static final double kCLIMB_V = 0;
    public static final double kCLIMB_A = 0;
    public static final double kCLIMB_G = 0;

    public static final int LEADERCLIMBID = 22;
    // public static final int FOLLOWERCLIMBID = 0;

  }

  public static class DriveToPIDConstants {
    // Translation (X/Y) PID - output is m/s per meter of error
    public static final double TRANSLATION_KP = 2.0;
    public static final double TRANSLATION_KD = 0.0;
    // Rotation PID - output is rad/s per radian of error
    public static final double ROTATION_KP    = 3.0;
    public static final double ROTATION_KD    = 0.0;
    // Tolerances
    public static final double TRANSLATION_TOLERANCE_M   = 0.05; // 5 cm
    public static final double ROTATION_TOLERANCE_RAD    = Units.degreesToRadians(2); // 2 degrees
  }

  public static class driveToPoseConstants {
    public static final Translation2d BlueLeftTranslation = new Translation2d(Meter.of(3),Meter.of(6));
    public static final Translation2d BlueCenterTranslation = new Translation2d(Meter.of(3),Meter.of(4));
    public static final Translation2d BlueRightTranslation = new Translation2d(Meter.of(3),Meter.of(2));
    public static final Translation2d BlueBackRightTranslation = new Translation2d(Meter.of(16),Meter.of(2));
    public static final Translation2d BlueBackLeftTranslation = new Translation2d(Meter.of(16),Meter.of(6));

    public static final Rotation2d BlueLeftRotation = new Rotation2d(0);
    public static final Rotation2d BlueCenterRotation = new Rotation2d(0);
    public static final Rotation2d BlueRightRotation = new Rotation2d(0);
    public static final Rotation2d BlueBackRightRotation = new Rotation2d(0);
    public static final Rotation2d BlueBackLeftRotation = new Rotation2d(0);

    public static final Pose2d BLUELEFTPOSE2D = new Pose2d(BlueLeftTranslation, BlueLeftRotation);
    public static final Pose2d BLUECENTERPOSE2D = new Pose2d(BlueCenterTranslation, BlueCenterRotation);
    public static final Pose2d BLUERIGHTPOSE2D = new Pose2d(BlueRightTranslation, BlueRightRotation);
    public static final Pose2d BLUEBACKRIGHTPOSE2D = new Pose2d(BlueBackRightTranslation, BlueBackRightRotation);
    public static final Pose2d BLUEBACKLEFTPOSE2D = new Pose2d(BlueBackLeftTranslation, BlueBackLeftRotation);

    public static final Translation2d RedLeftTranslation = new Translation2d(Meter.of(14),Meter.of(3));
    public static final Translation2d RedCenterTranslation = new Translation2d(Meter.of(14),Meter.of(4));
    public static final Translation2d RedRightTranslation = new Translation2d(Meter.of(14),Meter.of(5));
    public static final Translation2d RedBackRightTranslation = new Translation2d(Meter.of(15),Meter.of(6));
    public static final Translation2d RedBackLeftTranslation = new Translation2d(Meter.of(15),Meter.of(2));

    public static final Rotation2d RedLeftRotation = new Rotation2d(0);
    public static final Rotation2d RedCenterRotation = new Rotation2d(0);
    public static final Rotation2d RedRightRotation = new Rotation2d(0);
    public static final Rotation2d RedBackRightRotation = new Rotation2d(0);
    public static final Rotation2d RedBackLeftRotation = new Rotation2d(0);


    public static final Pose2d REDLEFTPOSE2D = new Pose2d(RedLeftTranslation, RedLeftRotation);
    public static final Pose2d REDCENTERPOSE2D = new Pose2d(RedCenterTranslation, RedCenterRotation);
    public static final Pose2d REDRIGHTPOSE2D = new Pose2d(RedRightTranslation, RedRightRotation);
    public static final Pose2d REDBACKRIGHTPOSE2D = new Pose2d(RedBackRightTranslation, RedBackRightRotation);
    public static final Pose2d REDBACKLEFTPOSE2D = new Pose2d(RedBackLeftTranslation, RedBackLeftRotation);


    public static final double SHOOTERPOS1RPM = 500;
    public static final double SHOOTERPOS2RPM = 500;
    public static final double SHOOTERPOS3RPM = 500;
    public static final double SHOOTERPOS4RPM = 500;
    public static final double SHOOTERPOS5RPM = 500;
    
  }
}
