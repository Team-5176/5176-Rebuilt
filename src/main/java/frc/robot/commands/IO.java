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
  Trigger reverseIntakeButton = new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.5);
  JoystickButton intakePivotToggleButton = new JoystickButton(driverXbox, 6);
  JoystickButton reverseTransportAndSpin = new JoystickButton(driverXbox, 7);
  JoystickButton reverseSpindexTransportButton = new JoystickButton(driverXbox, 3);

  Trigger position1Button = new Trigger(() -> driverXbox.getPOV() == 0);
  Trigger position2Button = new Trigger(()-> driverXbox.getPOV() == 180);
  // Trigger position0Button = new Trigger(() -> driverXbox.getPOV() == 180);
  
  // JoystickButton toggleSpindexerButton = new JoystickButton(driverXbox,  3);
  // JoystickButton shootOnlyButton = new JoystickButton(driverXbox, 2);

  // LED Button
  // JoystickButton ledoff = new JoystickButton(operatorXbox, 4)    .whenPressed(m_turnOnLEDsCommand);
    
  public IO() {
    shootButton.onTrue(RebuiltCommands.getToggleShoot());
    // Run intake only while the intake button is held.
    // intakeButton.whileTrue(RebuiltCommands.toggleIntake);
    reverseTransportAndSpin.onTrue(RebuiltCommands.getReverseTransportAndSpin());
    // Hold X to reverse the spindexer/transport to clear a jam; on release, resume
    // whatever the shoot toggle (A) currently says (forward if it's on, stopped if not).
    reverseSpindexTransportButton.whileTrue(RebuiltCommands.getReverseSpindexAndTransport());
    reverseSpindexTransportButton.onFalse(RebuiltCommands.getResumeSpindexAndTransport());
    reverseIntakeButton.onTrue(RebuiltCommands.getToggleReverseIntake());
    intakeButton.onTrue(RebuiltCommands.getToggleIntake());
    position1Button.onTrue(RebuiltCommands.topPos);
    position2Button.onTrue(RebuiltCommands.bottomPos);
    intakePivotToggleButton.onTrue(RebuiltCommands.getAngleIntake());

      // shootButton.onFalse(RebuiltCommands.toggleShoot);
      // toggleIntakeButton.toggleOnTrue(RebuiltCommands.startIntake);
      // toggleTransporButton.toggleOnTrue(RebuiltCommands.toggleTransport);
      // toggleSpindexerButton.toggleOnTrue(RebuiltCommands.toggleSpindex);
      // shootOnlyButton.onTrue(RebuiltCommands.shootFuel);
    }
}
