package frc.robot.commands;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;

// "() ->", or lambda function, allows to put in a method value where otherwise unable

public class RebuiltCommands {

    private static Command getShootFuel() {
        return new InstantCommand(
            () -> Robot.shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM),
            Robot.shooterSubsystem
        );
    }

    private static Command getStopShoot() {
        return new InstantCommand(() -> Robot.shooterSubsystem.setShooterVelocity(0), Robot.shooterSubsystem);
    }
    
    private static Command getStartTransport() {
        return new InstantCommand(
            () -> Robot.transportSubsystem.setTransport(Constants.TransportConstants.TRANSPORT_VELOCITY_RPM),
            Robot.transportSubsystem
        );
    }

    private static Command getStopTransport() {
        return new InstantCommand(() -> Robot.transportSubsystem.setTransport(0), Robot.transportSubsystem);
    }

    private static Command getStartSpindexer() {
        return new InstantCommand(
            () -> Robot.spindexerSubsystem.runSpindexer(Constants.SpindexerConstants.SPINDEXER_TARGET_VELOCITY_RPM),
            Robot.spindexerSubsystem
        );
    }

    private static Command getStopSpindexer() {
        return new InstantCommand(() -> Robot.spindexerSubsystem.runSpindexer(0.0), Robot.spindexerSubsystem);
    }

    private static Command getDeployIntake() {
        return new InstantCommand(
            () -> Robot.intakeSubsystem.deployIntake(Constants.IntakeConstants.kArmRotations),
            Robot.intakeSubsystem
        );
    }

    private static Command getRetractIntake() {
        return new InstantCommand(
            () -> Robot.intakeSubsystem.retractIntake(Constants.IntakeConstants.kArmRetractPos),
            Robot.intakeSubsystem
        );
    }
    
    public static final Command bottomPos = new InstantCommand(
        ()-> Robot.towerClimbSubsystem.setTowerClimbPosition(Constants.TowerConstants.CLIMBPOS), Robot.towerClimbSubsystem);
    public static final Command topPos = new InstantCommand(
        ()-> Robot.towerClimbSubsystem.setTowerClimbPosition(Constants.TowerConstants.RESETPOS), Robot.towerClimbSubsystem);
        
    public static final Command towerDown = new InstantCommand(
        () -> Robot.towerClimbSubsystem.setTowerClimbVelocity(0.1), Robot.towerClimbSubsystem
    );
    public static final Command towerStop = new InstantCommand(
        () -> Robot.towerClimbSubsystem.setTowerClimbVelocity(0), Robot.towerClimbSubsystem
    );
    // public static final Command bottomPos = new InstantCommand(
    //     ()-> Robot.towerClimbSubsystem.setTowerClimbPosition(0), Robot.towerClimbSubsystem);

    public static final Command startRumble = new InstantCommand(()->IO.driverXbox.setRumble(RumbleType.kBothRumble, 1.0));
    public static final Command stopRumble = new InstantCommand(()->IO.driverXbox.setRumble(RumbleType.kBothRumble, 0.0));
    public static Command getStartIntake() {
        return Commands.runOnce(() -> Robot.intakeSubsystem.spinIntake(Constants.IntakeConstants.INTAKE_ROLLER_VELOCITY_RPM), Robot.intakeSubsystem);
    }

    public static Command getStopIntake() {
        return Commands.runOnce(
            () -> Robot.intakeSubsystem.spinIntake(0.0), Robot.intakeSubsystem);
    }

    public static Command getReverseIntake() {
        return Commands.runOnce(
            () -> Robot.intakeSubsystem.spinIntake(-Constants.IntakeConstants.INTAKE_ROLLER_VELOCITY_RPM), Robot.intakeSubsystem);
    }
    // public static final Command startIntake = new InstantCommand(()-> Robot.intakeSubsystem.spinIntake(Constants.IntakeConstants.INTAKE_ROLLER_VELOCITY_RPM), Robot.intakeSubsystem);
    // public static final Command stopIntake = new InstantCommand(()-> Robot.intakeSubsystem.spinIntake(0.0), Robot.intakeSubsystem);


    // Cannot stop shooting on button press until WaitCommands finish
    public static Command getToggleShoot() {
        return new ConditionalCommand(
            getStopShoot().andThen(getStopTransport()).andThen(getStopSpindexer()),
            getShootFuel().andThen(new WaitCommand(2.0)).andThen(getStartTransport()).andThen(new WaitCommand(1.0)).andThen(getStartSpindexer()),
            Robot.shooterSubsystem::isShooting
        );
    }

    // Run the intake rollers while the button is held.
    // does this work for starting and stopping the intake?  
    // public static final Command toggleIntake = new ConditionalCommand(
    //     stopIntake,
    //     startIntake,
    //     Robot.intakeSubsystem::isIntaking
    // );

    public static Command getToggleIntake() {
        return new ConditionalCommand(
            getStopIntake(),
            getStartIntake(),
            Robot.intakeSubsystem::isIntaking
        );
    }
    public static Command getToggleReverseIntake() {
        return new ConditionalCommand(
            getStopIntake(), 
            getReverseIntake(), 
            Robot.intakeSubsystem::isIntaking
            );
    }
    

    public static Command getAngleIntake() {
        return new ConditionalCommand(
            getRetractIntake(),
            getDeployIntake(),
            Robot.intakeSubsystem::isDeployed
        );
    }

    //  public static final ConditionalCommand toggleSpindex = new ConditionalCommand(
    //     stopSpindexer,
    //     startSpindexer,
    //     Robot.spindexerSubsystem::isSpindexing
    // );

    // public static final SequentialCommandGroup rumble = new SequentialCommandGroup(
    //     startRumble,
    //     new WaitCommand(1),
    //     stopRumble
    // );


    // public static final Command turnOnLED = new InstantCommand(
    // () -> Robot.ledsubsystem.setAllLedsToColor(255, 0 , 0));
}
