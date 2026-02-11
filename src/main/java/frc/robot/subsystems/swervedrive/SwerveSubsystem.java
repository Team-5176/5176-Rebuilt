// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class SwerveSubsystem extends SubsystemBase
{
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;
  private Vision vision;

  public SwerveSubsystem(){
    try {
        swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.MAX_SPEED,
                                                            new Pose2d(new Translation2d(Meter.of(0),
                                                                                          Meter.of(0)),
                                                                                          Rotation2d.fromDegrees(0)));
        // Alternative method if you don't want to supply the conversion factor via JSON files.
        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
      } catch (Exception e) {
        throw new RuntimeException(e);
      }
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  }

  public SwerveDrive getSwerveDrive() {
      return swerveDrive;
  }

    /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision()
  {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }
  // This method checks the current alliance color from the DriverStation and returns true if it's the Red alliance, false otherwise. If the alliance information is not available, it defaults to false (Blue Alliance).
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  // This method calculates the heading of the robot based on its current velocity vector. It returns the field oriented heading in degrees.
  public Rotation2d getHeading() {
    ChassisSpeeds fieldVelocity = swerveDrive.getFieldVelocity();

    // Calculate the direction angle from the velocity components
    double velocityDirection = Math.atan2(fieldVelocity.vyMetersPerSecond, 
                                          fieldVelocity.vxMetersPerSecond);
    // Convert to degrees if needed
    double velocityDirectionDegrees = Math.toDegrees(velocityDirection);

    Rotation2d velocityRotation2d = new Rotation2d(velocityDirectionDegrees);

    return velocityRotation2d;
  }
  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
          } else
          {
            zeroGyro();
          }
  }
  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }
      
    public Pose2d getPose() {
      return swerveDrive.getPose();
    }
}
