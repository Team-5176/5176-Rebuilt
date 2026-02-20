package frc.robot.commands;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.IO;

// "() ->", or lambda function, allows to put in a method value where otherwise unable

public class RebuiltCommands {


    public static final Command shootFuel = new InstantCommand(()-> Robot.shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM), Robot.shooterSubsystem);
    public static final Command stopShoot = new InstantCommand(()-> Robot.shooterSubsystem.setShooterVelocity(0), Robot.shooterSubsystem);
    
    public static final Command runSpindexer = new InstantCommand(()-> Robot.spindexerSubsystem.runSpindexer(0.50), Robot.spindexerSubsystem);
    public static final Command stopSpindexer = new InstantCommand(()-> Robot.spindexerSubsystem.runSpindexer(0.0), Robot.spindexerSubsystem);

    public static final Command startIntake = new InstantCommand(()-> Robot.intakeSubsystem.spinIntake(0.5), Robot.intakeSubsystem);
    public static final Command stopIntake = new InstantCommand(()-> Robot.intakeSubsystem.spinIntake(0.0), Robot.intakeSubsystem);

    public static final Command deployIntake = new InstantCommand(()-> Robot.intakeSubsystem.deployIntake(0.5) ,Robot.intakeSubsystem);
    public static final Command retractIntake = new InstantCommand(()-> Robot.intakeSubsystem.deployIntake(0), Robot.intakeSubsystem);

    public static final Command startTransport = new InstantCommand(()-> Robot.transportSubsystem.setTransport(0.5), Robot.transportSubsystem);
    public static final Command stopTransport = new InstantCommand(()-> Robot.transportSubsystem.setTransport(0), Robot.transportSubsystem);
    
    public static final Command climberTopPose = new InstantCommand(()-> Robot.towerClimbSubsystem.setTowerClimbPosition(2), Robot.towerClimbSubsystem);
    public static final Command climberBottomPose = new InstantCommand(()-> Robot.towerClimbSubsystem.setTowerClimbPosition(0), Robot.towerClimbSubsystem);

    public static final Command startRumble = new InstantCommand(()->IO.driverXbox.setRumble(RumbleType.kBothRumble, 1.0));
    public static final Command stopRumble = new InstantCommand(()->IO.driverXbox.setRumble(RumbleType.kBothRumble, 0.0));


    public static final ConditionalCommand toggleShoot = new ConditionalCommand(
        stopShoot.andThen(stopSpindexer).andThen(stopTransport),
        shootFuel.andThen(runSpindexer).andThen(startTransport),
        Robot.shooterSubsystem::isShooting
    );

    public static final ConditionalCommand toggleIntake = new ConditionalCommand(
        stopIntake,
        startIntake,
        Robot.intakeSubsystem::isIntaking
    );

    public static final SequentialCommandGroup readyClimber = new SequentialCommandGroup(
        retractIntake,
        new WaitCommand(2),
        deployClimber
    );

    public static final SequentialCommandGroup rumble = new SequentialCommandGroup(
        startRumble,
        new WaitCommand(1),
        stopRumble
    );




    //many other cool possibilities with "WaitUntilCommand"
    //many other cool possibilities with limit switches

    // LED Command Code
    // Create an LED pattern that sets the entire strip to solid red
    // public static final Command turnOnLED = new InstantCommand(
    // () -> Robot.ledsubsystem.setAllLedsToColor(255, 0 , 0));
}
