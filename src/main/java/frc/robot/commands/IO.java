package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


// Button numbers on the controller:
// https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.chiefdelphi.com%2Ft%2Fhow-to-program-an-xbox-controller-to-drive-a-robot%2F131164&psig=AOvVaw28II86to-llZYujh--NhGp&ust=1759627474419000&source=images&cd=vfe&opi=89978449&ved=0CBkQjhxqFwoTCIC8oKaxiZADFQAAAAAdAAAAABAE

//IO means Input/Output
public class IO {

  public static XboxController driverXbox = new XboxController(0);

  JoystickButton shootButton = new JoystickButton(driverXbox,  1);
  JoystickButton intakeButton = new JoystickButton(driverXbox, 2);
  JoystickButton intakeDeployButton = new JoystickButton(driverXbox, 5);
  JoystickButton intakeRetractButton = new JoystickButton(driverXbox, 6);
  Trigger position1Button = new Trigger(() -> driverXbox.getPOV() == 0);
  // Trigger position0Button = new Trigger(() -> driverXbox.getPOV() == 180);
  
  // JoystickButton toggleSpindexerButton = new JoystickButton(driverXbox,  3);
  // JoystickButton shootOnlyButton = new JoystickButton(driverXbox, 2);
  // JoystickButton toggleIntakeButton = new JoystickButton(driverXbox,  5); // Use button 5 (left bumper) for intake toggle

  // LED Button
  // JoystickButton ledoff = new JoystickButton(operatorXbox, 4)    .whenPressed(m_turnOnLEDsCommand);
    
  public IO() {
    shootButton.onTrue(RebuiltCommands.toggleShoot);
    // Run intake only while the intake button is held.
    intakeButton.whileTrue(RebuiltCommands.intakeWhileHeld);
    // position1Button.whileTrue(RebuiltCommands.topPos);
    intakeDeployButton.onTrue(RebuiltCommands.deployIntake);
    intakeRetractButton.onTrue(RebuiltCommands.retractIntake);

    // intakeAngleButton.onTrue(RebuiltCommands.angleIntake);
      
      // shootButton.onFalse(RebuiltCommands.toggleShoot);
      // toggleIntakeButton.toggleOnTrue(RebuiltCommands.startIntake);
      // toggleTransporButton.toggleOnTrue(RebuiltCommands.toggleTransport);
      // toggleSpindexerButton.toggleOnTrue(RebuiltCommands.toggleSpindex);
      // shootOnlyButton.onTrue(RebuiltCommands.shootFuel);
    }
}
