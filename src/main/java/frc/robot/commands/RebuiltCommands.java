package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;

// "() ->", or lambda function, allows to put in a method value where otherwise unable

public class RebuiltCommands {


    public static final Command shootFuel = new InstantCommand(()-> Robot.shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM), Robot.shooterSubsystem);
    public static final Command stopShoot = new InstantCommand(()-> Robot.shooterSubsystem.setShooterVelocity(0), Robot.shooterSubsystem);
    
    public static final Command runSpindexer = new InstantCommand(()-> Robot.spindexerSubsystem.runSpindexer(0.50), Robot.spindexerSubsystem);
    public static final Command stopSpindexer = new InstantCommand(()-> Robot.spindexerSubsystem.runSpindexer(0.0), Robot.spindexerSubsystem);

    public static final Command startIntake = new InstantCommand(()-> Robot.intakeSubsystem.spinIntake(0.5), Robot.intakeSubsystem);
    public static final Command stopIntake = new InstantCommand(()->Robot.intakeSubsystem.spinIntake(0.0),Robot.intakeSubsystem);
   
    public static final ConditionalCommand toggleShoot = new ConditionalCommand(
        stopShoot.andThen(stopSpindexer),
        shootFuel.andThen(runSpindexer),
        Robot.shooterSubsystem::isShooting
    );
    
    public static final ConditionalCommand toggleIntake = new ConditionalCommand(
        stopIntake,
        startIntake,
        Robot.intakeSubsystem::isIntaking
    );



    //many other cool possibilities with "WaitUntilCommand"
    //many other cool possibilities with limit switches

    // LED Command Code
    // Create an LED pattern that sets the entire strip to solid red
    // public static final Command turnOnLED = new InstantCommand(
    // () -> Robot.ledsubsystem.setAllLedsToColor(255, 0 , 0));
}
