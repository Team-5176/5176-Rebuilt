// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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
  SwerveSubsystem drivebase = new SwerveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    DriverStation.silenceJoystickConnectionWarning(true);
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
  
  private void configureBindings()
  {}

}


