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

    public void retractHood(){
        if (getHoodEncoder() >= minEncoderTicks && getHoodEncoder() <= maxEncoderTicks) {
            this.hoodMotor.set(ControlMode.PercentOuput, throttle);
        }
        else if (getHoodEncode() < minEncoderTicks){
            this.hoodMotor.set(ControlMode.PercentOuput, 0.1);
        }
        else if (getHoodEncode() > maxEncoderTicks){
            this.hoodMotor.set(ControlMode.PercentOuput, -0.1);
        }
    }

    public Periodic(){
        SmartDashBoard.putNumber("HoodMotor EncoderValue", getHoodEncoder());
    }

    public void turnToAngle(){
        if (this.hoodJoystick.getRawButton(3))
            if (getHoodEncoder() != 0){
                this.hoodMotor.set(ControlMode.PercentOuput, getHoodEncoder()/-1000);
            }
        else {
            this.hoodMotor.set(ControlMode.PercentOuput, 0);
        }
    }

    public void setAngle(double angle) {
        double currentAngle = getEncoder() * this.encoderDegree;
        // PID Control
        double error = angle - this.currentAngle;
        double turn = (this.kP * error);
        this.runMotor(turn * 0.5);
    }

    public void resetAngle(){
        setAngle(0)
    }
}