package frc.team1414.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team1414.robot.commands.Drivetrain.DriveCmd;

public class Drivetrain extends Subsystem {

    //Initialize speed controllers
    private TalonSRX leftDriveMaster = new TalonSRX(0);
    private VictorSPX leftDriveSlave = new VictorSPX(1);
    private TalonSRX rightDriveMaster = new TalonSRX(2);
    private VictorSPX rightDriveSlave = new VictorSPX(3);

    public Timer timer = new Timer();

    public Drivetrain() {
        //Set talon values
        this.leftDriveSlave.follow(leftDriveMaster);
        this.rightDriveSlave.follow(rightDriveMaster);
    }

    public void drive(double powLeft, double powRight) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, powLeft);
        this.rightDriveMaster.set(ControlMode.PercentOutput, powRight);
    }

    public void driveArcade(double throttle, double turn) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, throttle + turn);
        this.rightDriveMaster.set(ControlMode.PercentOutput, throttle - turn);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new DriveCmd());
    }
}