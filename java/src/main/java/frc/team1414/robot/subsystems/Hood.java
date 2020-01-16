package frc.team1414.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Joystick;


public class Hood extends Subsystem {

    //Initialize speed controllers
    public TalonSRX hoodMotor = new TalonSRX(0);
    public JoyStick hoodJoystick = new JoyStick(0);
    public double maxEncoderTicks = 1000;
    public double minEncoderTicks = -1000;
    public double specificAngle;

    public Hood(){
        this.hoodMotor.configSelectedFeedBackSensor(FeedbackDevice.IntergratedSensor, 0 , 0);
    }

    public double getHoodEncoder(){
        return this.hoodMotor.getSelectedFeedbackSensorPosition();
    }

    public void runMotor(double throttle) {
        if(this.maxEncoderTicks > getEncoder() && getEncoder() > this.minEncoderTicks) {
            this.turretMotor.set(ControlMode.PercentOutput, this.throttle);
        } else if (this.maxEncoderTicks < getHoodEncoder()){
            if(this.throttle > 0) {
                this.turretMotor.set(ControlMode.PercentOutput, 0);
            } else {
                this.turretMotor.set(ControlMode.PercentOutput, this.throttle);
            }
        } else if (-this.maxEncoderTicks > getHoodEncoder()) {
            if(this.throttle > 0) {
                this.turretMotor.set(ControlMode.PercentOutput, this.throttle);
            } else {
                this.turretMotor.set(ControlMode.PercentOutput, 0);
            }
        }
    }

    public Periodic(){
        SmartDashBoard.putNumber("HoodMotor EncoderValue", getHoodEncoder());
    }

    public void setAngle(double angle) {
        double currentAngle = getEncoder() * this.encoderDegree;
        // PID Control
        double error = angle - currentAngle;
        double turn = (this.kP * error);
        this.runMotor(turn * 0.5);
    }

    public void resetAngle(){
        setAngle(0)
    }
}
