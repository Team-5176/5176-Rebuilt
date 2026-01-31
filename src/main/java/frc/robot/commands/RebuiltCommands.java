package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

// "() ->", or lambda function, allows to put in a method value where otherwise unable

public class RebuiltCommands {


         public static final Command shootFuel = new InstantCommand(()-> Robot.shooterSubsystem.runRoller(0.3), Robot.shooterSubsystem);



    //many other cool possibilities with "WaitUntilCommand"
    //many other cool possibilities with limit switches

    // LED Command Code
    // Create an LED pattern that sets the entire strip to solid red
    // public static final Command turnOnLED = new InstantCommand(
    // () -> Robot.ledsubsystem.setAllLedsToColor(255, 0 , 0));
}
