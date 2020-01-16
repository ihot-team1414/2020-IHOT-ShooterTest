package frc.team1414.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.team1414.robot.Robot;

public class DriveCmd extends Command {
    public DriveCmd() {
        requires(Robot.drivetrain);
    }

    protected void initialize() {
        //Initialize subsystem modes
    }

    protected void execute() {
        Robot.drivetrain.driveArcade(Robot.oi.controller.getY(), Robot.oi.controller.getX());
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    }

    protected void interrupted() {
        super.interrupted();
    }
}