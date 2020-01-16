package frc.team1414.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class OI 
{
     //Initialize controllers here
     public Joystick controller = new Joystick(0);

     //Initialize controller buttons here
     public Button button = new JoystickButton(controller, 1);

    public OI() {
        //Connect buttons to commands with helper methods here
        button.whenPressed(new WaitCommand(0));
    }
}