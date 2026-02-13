
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

    public XboxController operatorXbox = new XboxController(1);

    // Trigger shootButton = new Trigger(() -> operatorXbox.getRightTriggerAxis() > 0.7);//XboxControl..........Right Trigger
    JoystickButton toggleShootButton = new JoystickButton(operatorXbox, 1);//XboxController...A
    JoystickButton toggleIntakeButton = new JoystickButton(operatorXbox, 2);//XboxController...B
    // LED Button
    // JoystickButton ledoff = new JoystickButton(operatorXbox, 4)    .whenPressed(m_turnOnLEDsCommand);
    
    public IO() {
        // whiletrue buttons here

        toggleShootButton.toggleOnTrue(RebuiltCommands.toggleShoot);
        toggleIntakeButton.toggleOnTrue(RebuiltCommands.toggleIntake);
    }


}
