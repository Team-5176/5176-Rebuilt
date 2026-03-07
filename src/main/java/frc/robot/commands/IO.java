
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Button numbers on the controller:
// https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.chiefdelphi.com%2Ft%2Fhow-to-program-an-xbox-controller-to-drive-a-robot%2F131164&psig=AOvVaw28II86to-llZYujh--NhGp&ust=1759627474419000&source=images&cd=vfe&opi=89978449&ved=0CBkQjhxqFwoTCIC8oKaxiZADFQAAAAAdAAAAABAE

//IO means Input/Output
public class IO {
    //create xbox controller objects

    // public XboxController driverXbox = new XboxController(0); add back????

    // Driver Xbox controller. Port 0 is the usual driver station index; change if your controller is on a different port.
    public static XboxController driverXbox = new XboxController(0);


    JoystickButton shootButton = new JoystickButton(driverXbox,  1);
  JoystickButton intakeButton = new JoystickButton(driverXbox, 5);
    // JoystickButton toggleSpindexerButton = new JoystickButton(driverXbox,  3);
    // JoystickButton shootOnlyButton = new JoystickButton(driverXbox, 2);
   // JoystickButton toggleIntakeButton = new JoystickButton(driverXbox,  5); // Use button 5 (left bumper) for intake toggle

    // LED Button
    // JoystickButton ledoff = new JoystickButton(operatorXbox, 4)    .whenPressed(m_turnOnLEDsCommand);
    
    public IO() {
        // whiletrue buttons here

      shootButton.onTrue(RebuiltCommands.toggleShoot);
      // Run intake only while the intake button is held
      intakeButton.whileTrue(RebuiltCommands.intakeWhileHeld);
        // shootButton.onFalse(RebuiltCommands.toggleShoot);

      // toggleIntakeButton.toggleOnTrue(RebuiltCommands.startIntake);
        // toggleTransporButton.toggleOnTrue(RebuiltCommands.toggleTransport);
        // toggleSpindexerButton.toggleOnTrue(RebuiltCommands.toggleSpindex);
        // shootOnlyButton.onTrue(RebuiltCommands.shootFuel);
    }


}
