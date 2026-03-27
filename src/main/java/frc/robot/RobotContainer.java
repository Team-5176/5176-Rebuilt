// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer
{
  private static final String SHOOTER_DEFAULT_RPM_KEY = "Shooter RPM/Default";
  private static final String SHOOTER_LEFT_RPM_KEY = "Shooter RPM/Left";
  private static final String SHOOTER_CENTER_RPM_KEY = "Shooter RPM/Center";
  private static final String SHOOTER_RIGHT_RPM_KEY = "Shooter RPM/Right";
  private static final String SHOOTER_ACTIVATION_RADIUS_KEY = "Shooter RPM/Activation Radius M";
  private static final String SHOOTER_ACTIVE_POSE_KEY = "Shooter RPM/Active Pose";
  private static final String SHOOTER_DISTANCE_KEY = "Shooter RPM/Distance To Active Pose";

  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final CommandXboxController driverXbox = new CommandXboxController(0);

    // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;
  // The container for the robot. Contains subsystems, OI devices, and commands.
  
  public RobotContainer()
  {
    initializeShooterDashboard();
    configureDriveToPose();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    DriverStation.silenceJoystickConnectionWarning(true);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                    () -> -driverXbox.getLeftY(),
                                                                    () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRightX())
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
                                                                    
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                           .withControllerHeadingAxis((((driverXbox::getRightX))),
                                                                                         driverXbox::getRightY)
                                                           .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  private void configureDriveToPose()
  {
    driverXbox.x().whileTrue(driveToAlliancePoseCommand(
        new Pose2d(new Translation2d(2.6, 6), Rotation2d.fromDegrees(130)),
        new Pose2d(new Translation2d(14, 3), Rotation2d.fromDegrees(-50))));
    driverXbox.y().whileTrue(driveToAlliancePoseCommand(
        new Pose2d(new Translation2d(2.6, 4), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(14, 4), Rotation2d.fromDegrees(180))));
    driverXbox.b().whileTrue(driveToAlliancePoseCommand(
        new Pose2d(new Translation2d(2.6, 2), Rotation2d.fromDegrees(-130)),
        new Pose2d(new Translation2d(14, 5), Rotation2d.fromDegrees(50))));
  }

  private Command driveToAlliancePoseCommand(Pose2d bluePose, Pose2d redPose)
  {
    return Commands.defer(
        () -> Commands.either(
                drivebase.driveToPosePID(redPose),
                drivebase.driveToPosePID(bluePose),
                drivebase::isRedAlliance),
        Set.of(drivebase));
  }

  public void periodic()
  {
    updateShooterTargetRPM();
  }

  private void initializeShooterDashboard()
  {
    SmartDashboard.putNumber(SHOOTER_DEFAULT_RPM_KEY, Constants.ShooterConstants.DEFAULT_TARGET_VELOCITY_RPM);
    SmartDashboard.putNumber(SHOOTER_LEFT_RPM_KEY, Constants.driveToPoseConstants.SHOOTER_LEFT_RPM);
    SmartDashboard.putNumber(SHOOTER_CENTER_RPM_KEY, Constants.driveToPoseConstants.SHOOTER_CENTER_RPM);
    SmartDashboard.putNumber(SHOOTER_RIGHT_RPM_KEY, Constants.driveToPoseConstants.SHOOTER_RIGHT_RPM);
    SmartDashboard.putNumber(SHOOTER_ACTIVATION_RADIUS_KEY,
                             Constants.driveToPoseConstants.SHOOTER_POSE_ACTIVATION_RADIUS_M);
    SmartDashboard.putNumber("Shooter Target RPM", Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM);
    SmartDashboard.putString(SHOOTER_ACTIVE_POSE_KEY, "Default");
    SmartDashboard.putNumber(SHOOTER_DISTANCE_KEY, -1.0);
  }

  private void updateShooterTargetRPM()
  {
    double defaultRPM = SmartDashboard.getNumber(SHOOTER_DEFAULT_RPM_KEY,
                                                 Constants.ShooterConstants.DEFAULT_TARGET_VELOCITY_RPM);
    double activationRadiusM = SmartDashboard.getNumber(
        SHOOTER_ACTIVATION_RADIUS_KEY,
        Constants.driveToPoseConstants.SHOOTER_POSE_ACTIVATION_RADIUS_M);

    PoseRPMChoice closestChoice = getClosestPoseRPMChoice();
    boolean usePoseRPM = closestChoice.distanceMeters <= activationRadiusM;
    double targetRPM = usePoseRPM ? closestChoice.rpm : defaultRPM;

    Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM = targetRPM;

    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
    SmartDashboard.putString(SHOOTER_ACTIVE_POSE_KEY, usePoseRPM ? closestChoice.name : "Default");
    SmartDashboard.putNumber(SHOOTER_DISTANCE_KEY, closestChoice.distanceMeters);
  }

  private PoseRPMChoice getClosestPoseRPMChoice()
  {
    Pose2d currentPose = drivebase.getPose();

    PoseRPMChoice[] candidates = drivebase.isRedAlliance()
        ? new PoseRPMChoice[] {
            createPoseRPMChoice("Left", Constants.driveToPoseConstants.REDLEFTPOSE2D, SHOOTER_LEFT_RPM_KEY, currentPose),
            createPoseRPMChoice("Center", Constants.driveToPoseConstants.REDCENTERPOSE2D, SHOOTER_CENTER_RPM_KEY, currentPose),
            createPoseRPMChoice("Right", Constants.driveToPoseConstants.REDRIGHTPOSE2D, SHOOTER_RIGHT_RPM_KEY, currentPose)
          }
        : new PoseRPMChoice[] {
            createPoseRPMChoice("Left", Constants.driveToPoseConstants.BLUELEFTPOSE2D, SHOOTER_LEFT_RPM_KEY, currentPose),
            createPoseRPMChoice("Center", Constants.driveToPoseConstants.BLUECENTERPOSE2D, SHOOTER_CENTER_RPM_KEY, currentPose),
            createPoseRPMChoice("Right", Constants.driveToPoseConstants.BLUERIGHTPOSE2D, SHOOTER_RIGHT_RPM_KEY, currentPose)
          };

    PoseRPMChoice bestChoice = candidates[0];
    for (int i = 1; i < candidates.length; i++)
    {
      if (candidates[i].distanceMeters < bestChoice.distanceMeters)
      {
        bestChoice = candidates[i];
      }
    }
    return bestChoice;
  }

  private PoseRPMChoice createPoseRPMChoice(String name, Pose2d pose, String dashboardKey, Pose2d currentPose)
  {
    return new PoseRPMChoice(
        name,
        SmartDashboard.getNumber(dashboardKey, Constants.ShooterConstants.DEFAULT_TARGET_VELOCITY_RPM),
        currentPose.getTranslation().getDistance(pose.getTranslation()));
  }

  private record PoseRPMChoice(String name, double rpm, double distanceMeters)
  {
  }
}

