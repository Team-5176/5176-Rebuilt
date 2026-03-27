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
  Trigger intakeButton = new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.5);
  // JoystickButton intakeDeployButton = new JoystickButton(driverXbox, 5);
  // JoystickButton intakeRetractButton = new JoystickButton(driverXbox, 6);
 JoystickButton intakeAngleButton = new JoystickButton(driverXbox, 6);

  Trigger climbUpButton   = new Trigger(() -> driverXbox.getPOV() == 0);   // DPAD UP   → extend to climb position
  Trigger climbDownButton = new Trigger(() -> driverXbox.getPOV() == 180); // DPAD DOWN → retract to reset position
  
  // JoystickButton toggleSpindexerButton = new JoystickButton(driverXbox,  3);
  // JoystickButton shootOnlyButton = new JoystickButton(driverXbox, 2);

  // LED Button
  // JoystickButton ledoff = new JoystickButton(operatorXbox, 4)    .whenPressed(m_turnOnLEDsCommand);
    
  public IO() {
    shootButton.onTrue(RebuiltCommands.toggleShoot);
    // Run intake only while the intake button is held.
    // intakeButton.whileTrue(RebuiltCommands.toggleIntake);
    intakeButton.onTrue(RebuiltCommands.getToggleIntake());
    climbUpButton.onTrue(RebuiltCommands.bottomPos);   // DPAD UP   → CLIMBPOS (73 rot)
    climbDownButton.onTrue(RebuiltCommands.topPos);    // DPAD DOWN → RESETPOS (0 rot)
    // intakeDeployButton.onTrue(RebuiltCommands.deployIntake);
    // intakeRetractButton.onTrue(RebuiltCommands.retractIntake);

    intakeAngleButton.onTrue(RebuiltCommands.angleIntake);
      
      // shootButton.onFalse(RebuiltCommands.toggleShoot);
      // toggleIntakeButton.toggleOnTrue(RebuiltCommands.startIntake);
      // toggleTransporButton.toggleOnTrue(RebuiltCommands.toggleTransport);
      // toggleSpindexerButton.toggleOnTrue(RebuiltCommands.toggleSpindex);
      // shootOnlyButton.onTrue(RebuiltCommands.shootFuel);
    }
}
