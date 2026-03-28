// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.File;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RebuiltCommands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerClimbSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */



public class RobotContainer
{
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final CommandXboxController driverXbox = new CommandXboxController(0);

    // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;
  // The container for the robot. Contains subsystems, OI devices, and commands.

  
  

  
  public RobotContainer()
  {
    configureDriveToPose();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    DriverStation.silenceJoystickConnectionWarning(true);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    drivebase.zeroGyroWithAlliance();
  }


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                    () -> driverXbox.getLeftY() * 1.0,
                                                                    () -> driverXbox.getLeftX() * 1.0)
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

  

 /*  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  } */

  private void configureDriveToPose() {
    // driverXbox.y().whileTrue(drivebase.driveToPosePID(drivebase.getClosestPoint()));
    if(drivebase.isRedAlliance()) {

      driverXbox.x().whileTrue(
          drivebase.driveToPosePID(new Pose2d(new Translation2d(14, 2), Rotation2d.fromDegrees(-50)))
          .alongWith(Commands.runOnce(() -> {
              // Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM = 1000;
              SmartDashboard.putNumber("Shooter Target RPM", Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM);
          }))
      );
      driverXbox.y().whileTrue(
          drivebase.driveToPosePID(new Pose2d(new Translation2d(14, 4), Rotation2d.fromDegrees(0)))
          .alongWith(Commands.runOnce(() -> {
              // Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM = 500;
              SmartDashboard.putNumber("Shooter Target RPM", Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM);
          }))
      );
      driverXbox.b().whileTrue(
          drivebase.driveToPosePID(new Pose2d(new Translation2d(14, 6), Rotation2d.fromDegrees(50)))
          .alongWith(Commands.runOnce(() -> {
              // Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM = 700;
              SmartDashboard.putNumber("Shooter Target RPM", Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM);
          }))
      );
      // driverXbox.x().whileTrue(
      //   drivebase.driveToPosePID(new Pose2d(new Translation2d(14, 2), Rotation2d.fromDegrees(-50))));
      // driverXbox.x().whileTrue(Commands.runOnce(()-> Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM = 1000));
      // driverXbox.y().whileTrue(
      //   drivebase.driveToPosePID(new Pose2d(new Translation2d(14, 4), Rotation2d.fromDegrees(0))));
      // driverXbox.b().whileTrue(
        // drivebase.driveToPosePID(new Pose2d(new Translation2d(14, 6), Rotation2d.fromDegrees(50))));
    }else{
      // driverXbox.x().whileTrue(
      //   drivebase.driveToPosePID(new Pose2d(new Translation2d(2.6, 6), Rotation2d.fromDegrees(130))));
      // driverXbox.y().whileTrue(
      //     drivebase.driveToPosePID(new Pose2d(new Translation2d(2.6, 4), Rotation2d.fromDegrees(180))));
      // driverXbox.b().whileTrue(
      //       drivebase.driveToPosePID(new Pose2d(new Translation2d(2.6, 2), Rotation2d.fromDegrees(-130))));

      driverXbox.x().whileTrue(
          drivebase.driveToPosePID(new Pose2d(new Translation2d(2.6, 6), Rotation2d.fromDegrees(130)))
          .alongWith(Commands.runOnce(() -> {
              // Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM = 1000;
              SmartDashboard.putNumber("Shooter Target RPM", Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM);
          }))
      );
      driverXbox.y().whileTrue(
          drivebase.driveToPosePID(new Pose2d(new Translation2d(2.6, 4), Rotation2d.fromDegrees(180)))
          .alongWith(Commands.runOnce(() -> {
              // Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM = 500;
              SmartDashboard.putNumber("Shooter Target RPM", Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM);
          }))
      );
      driverXbox.b().whileTrue(
          drivebase.driveToPosePID(new Pose2d(new Translation2d(2.6, 2), Rotation2d.fromDegrees(-130)))
          .alongWith(Commands.runOnce(() -> {
              // Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM = 700;
              SmartDashboard.putNumber("Shooter Target RPM", Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM);
          }))
      );
    }


//     if(!drivebase.isRedAlliance()) {
//         driverXbox.x().whileTrue(drivebase.driveToPosePID(Constants.driveToPoseConstants.BLUELEFTPOSE2D));
//         driverXbox.a().whileTrue(drivebase.driveToPosePID(
//           new Pose2d(Constants.driveToPoseConstants.BlueCenterTranslation, Constants.driveToPoseConstants.BlueCenterRotation)));
//         driverXbox.b().whileTrue(drivebase.driveToPosePID(
//           new Pose2d(Constants.driveToPoseConstants.BlueRightTranslation, Constants.driveToPoseConstants.BlueRightRotation)));
//       }else {
//         driverXbox.x().whileTrue(drivebase.driveToPosePID(
//           new Pose2d(Constants.driveToPoseConstants.RedLeftTranslation, Constants.driveToPoseConstants.RedLeftRotation)));
//         driverXbox.a().whileTrue(drivebase.driveToPosePID(
//           new Pose2d(Constants.driveToPoseConstants.BlueCenterTranslation, Constants.driveToPoseConstants.RedCenterRotation)));
//         driverXbox.b().whileTrue(drivebase.driveToPosePID(
//           new Pose2d(Constants.driveToPoseConstants.BlueRightTranslation, Constants.driveToPoseConstants.RedRightRotation)));
      // }
//     }
  }
}

