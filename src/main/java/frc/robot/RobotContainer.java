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
    NamedCommands.registerCommand("First Shoot Start", RebuiltCommands.getStartShootSequence());
    NamedCommands.registerCommand("First Shoot Stop", RebuiltCommands.getStopShootSequence());
    NamedCommands.registerCommand("Intake Out", RebuiltCommands.getAngleIntake());
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    drivebase.zeroGyroWithAlliance();
  }


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                    () -> driverXbox.getLeftY() * -1.0,
                                                                    () -> driverXbox.getLeftX() * -1.0)
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

  

  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autonomous command.
    return autoChooser.getSelected();
  }

  private void configureDriveToPose() {
    // Y is the only drive-to-pose button; left/right variants (previously X/B) were removed
    // so X is free to control the spindexer/transport reverse-while-held binding in IO.java.
    boolean isRed = drivebase.isRedAlliance();
    Pose2d  centerPose = isRed ? Constants.driveToPoseConstants.REDCENTERPOSE2D : Constants.driveToPoseConstants.BLUECENTERPOSE2D;

    driverXbox.y().whileTrue(
        drivebase.driveToPosePID(centerPose)
        .alongWith(Commands.runOnce(() -> {
            Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM = 1800;
            SmartDashboard.putNumber("Shooter Target RPM", Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM);
        }))
    );
  }
}

