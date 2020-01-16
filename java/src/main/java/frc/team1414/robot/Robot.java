package frc.team1414.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1414.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {

    // Initialize subsystems
    public static Drivetrain drivetrain = new Drivetrain();
    public static OI oi;  

    private enum Autos {
        Auto1,
        Auto2,
        Auto3,
    }

    Command autoCommand;
    SendableChooser autoSelector;

    //Initialize auto commands

    @Override
    public void robotInit() {
        //Setup subsystems
        oi = new OI();
        autoSelector = new SendableChooser();

        //Set subsystem values
        this.drivetrain.timer.start();

        //Create auto chooser objects
        autoSelector.addDefault("Auto 1", Autos.Auto1);
        autoSelector.addObject("Auto 2", Autos.Auto2);
        autoSelector.addObject("Auto 3", Autos.Auto3);

        //Print auto data to smart dashboard
        SmartDashboard.putData("Auto Selector", autoSelector);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() { 
        Scheduler.getInstance().run();
        updateSmartdashboard();
    }

    @Override
    public void autonomousInit() {
        //Set subsystem values for autonomous

        //Pick autonomous command to run
        if (autoSelector.getSelected().equals(Autos.Auto1)) {
            autoCommand = new WaitCommand(0);
        }

        //Start autonomous command
        autoCommand.start();
        
        //Print chosen autonomous command
        SmartDashboard.putString("Chosen Auto", autoCommand.toString());
    }

    @Override
    public void autonomousPeriodic() { 
        Scheduler.getInstance().run();
        updateSmartdashboard();
    }

    @Override
    public void teleopInit() {
        //Set subsystem values for teleop
    }

    @Override
    public void teleopPeriodic() { 
        Scheduler.getInstance().run();
        updateSmartdashboard();
    }

    @Override
    public void testInit() { }

    @Override
    public void testPeriodic() { }

    public void updateSmartdashboard() {
        //Print information to smart dashboard
        SmartDashboard.putNumber("Variable name", 1);
    }
}