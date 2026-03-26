package frc.robot.commands;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;

// "() ->", or lambda function, allows to put in a method value where otherwise unable

public class RebuiltCommands {

    public static final Command shootFuel = new InstantCommand(()-> Robot.shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM), Robot.shooterSubsystem);
    public static final Command stopShoot = new InstantCommand(()-> Robot.shooterSubsystem.setShooterVelocity(0), Robot.shooterSubsystem);
    
    public static final Command startTransport = new InstantCommand(()-> Robot.transportSubsystem.setTransport(Constants.TransportConstants.TRANSPORT_VELOCITY_RPM), Robot.transportSubsystem);
    public static final Command stopTransport = new InstantCommand(()-> Robot.transportSubsystem.setTransport(0), Robot.transportSubsystem);

    public static final Command startSpindexer = new InstantCommand(()-> Robot.spindexerSubsystem.runSpindexer(Constants.SpindexerConstants.SPINDEXER_TARGET_VELOCITY_RPM), Robot.spindexerSubsystem);
    public static final Command stopSpindexer = new InstantCommand(()-> Robot.spindexerSubsystem.runSpindexer(0.0), Robot.spindexerSubsystem);

    public static final Command deployIntake = new InstantCommand(()-> Robot.intakeSubsystem.deployIntake(Constants.IntakeConstants.kArmRotations) ,Robot.intakeSubsystem);
    public static final Command retractIntake = new InstantCommand(()-> Robot.intakeSubsystem.deployIntake(Constants.IntakeConstants.kArmRetractPos), Robot.intakeSubsystem);
    
    public static final Command topPos = new InstantCommand(
        ()-> Robot.towerClimbSubsystem.setTowerClimbPosition(Constants.TowerConstants.CLIMBDISTANCE), Robot.towerClimbSubsystem);
    // public static final Command bottomPos = new InstantCommand(
    //     ()-> Robot.towerClimbSubsystem.setTowerClimbPosition(0), Robot.towerClimbSubsystem);

    public static final Command startRumble = new InstantCommand(()->IO.driverXbox.setRumble(RumbleType.kBothRumble, 1.0));
    public static final Command stopRumble = new InstantCommand(()->IO.driverXbox.setRumble(RumbleType.kBothRumble, 0.0));

    public static final Command startIntake = new InstantCommand(()-> Robot.intakeSubsystem.spinIntake(Constants.IntakeConstants.INTAKE_ROLLER_VELOCITY_RPM), Robot.intakeSubsystem);
    public static final Command stopIntake = new InstantCommand(()-> Robot.intakeSubsystem.spinIntake(0.0), Robot.intakeSubsystem);


    // Cannot stop shooting on button press until WaitCommands finish
    public static final ConditionalCommand toggleShoot = new ConditionalCommand(
        stopShoot.andThen(stopTransport).andThen(stopSpindexer),
        shootFuel.andThen(new WaitCommand(4.0)).andThen(startTransport).andThen(new WaitCommand(1.0)).andThen(startSpindexer),
        Robot.shooterSubsystem::isShooting
    );

    // Run the intake rollers while the button is held.
    public static final Command toggleIntake = new ConditionalCommand(
        stopIntake,
        startIntake,
        Robot.intakeSubsystem::isIntaking
    );

    
    

    // public static final ConditionalCommand angleIntake = new ConditionalCommand(
    //     retractIntake,
    //     deployIntake,
    //     Robot.intakeSubsystem::isDeployed
    // );

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
