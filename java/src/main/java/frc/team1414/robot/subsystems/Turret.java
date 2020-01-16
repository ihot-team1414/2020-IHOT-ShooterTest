package frc.team1414.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Turret extends Subsystem {

    Double maxEncoderTicks;
    Double encoderDegree;
    Double degreeEncoder;


    private final double kP = 0.0375;
    private final double kD = 0.0;

    //Initialize speed controllers
    private TalonSRX turretMotor = new TalonSRX(0);
    private double turretEncoder = 0;

    public Turret() {
        //Set talon values
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.Quadrature, 1, 1);

    }

    public void runMotor(double throttle) {
        if(this.maxEncoderTicks > getEncoder() && getEncoder() > -this.maxEncoderTicks) {
            this.turretMotor.set(ControlMode.PercentOutput, this.throttle);
        } else if (this.maxEncoderTicks < getEncoder()){
            if(this.throttle > 0) {
                this.turretMotor.set(ControlMode.PercentOutput, 0);
            } else {
                this.turretMotor.set(ControlMode.PercentOutput, this.throttle);
            }
        } else if (-this.maxEncoderTicks > getEncoder()) {
            if(this.throttle > 0) {
                this.turretMotor.set(ControlMode.PercentOutput, this.throttle);
            } else {
                this.turretMotor.set(ControlMode.PercentOutput, 0);
            }
        }
    }

    public void setAngle(double angle) {
        double currentAngle = getEncoder() * this.encoderDegree;


        // PID Control
        double error = angle - this.currentAngle;
        double turn = (this.kP * error);
        this.runMotor(turn * 0.5);
    }
        
    public void resetAngle() {
        this.setAngle(0);
    }

    public void turn(double throttle) {
        runMotor(throttle);
    }

    public double getEncoder() {
        turretEncoder = turretMotor.getSelectedSensorPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Turret Encoder", getEncoder());
    }

    
}