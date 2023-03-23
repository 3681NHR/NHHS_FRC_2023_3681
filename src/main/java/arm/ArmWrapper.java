package arm;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;

import lib.interfaces.MotorInterface;

import com.revrobotics.CANSparkMax;

public class ArmWrapper {

    private Encoder EncoderArm;
    private Encoder EncoderCarriage;
    private MotorInterface MotorArm;
    private CANSparkMax MotorCarriage;

    /**
     * Wrapper for the arm, custom made for the 2023 season
     * 
     * @param armEncoder
     * @param carriageEncoder
     * @param armMotor
     * @param carriageMotor
     * 
     * @see ArmController
     */
    public ArmWrapper(Encoder armEncoder, Encoder carriageEncoder, MotorInterface armMotor, CANSparkMax carriageMotor) {
        EncoderArm = armEncoder;
        EncoderCarriage = carriageEncoder;
        MotorArm = armMotor;
        MotorCarriage = carriageMotor;
    }

    /**
     * @return Gets the encoder-found angle on the rotation axis for the big arm
     */
    public double getAngleArm() {
        return EncoderArm.getDistance();
    }

    /**
     * @return Gets the encoder-found angle on the rotation axis for the carriage
     */
    public double getAngleCarriage() {
        return EncoderCarriage.getDistance();
    }

    /**
     * @return Gets the encoder-found rate on the rotation axis for the arm
     */
    public double getRateArm() {
        return EncoderArm.getRate();
    }

    /**
     * @return Gets the encoder-found rate on the rotation axis for the carriage
     */
    public double getRateCarriage() {
        return EncoderCarriage.getRate();
    }

    /**
     * @return the output for the PID's
     */
    public double getPIDout(double setpoint) {
        double differencer = EncoderArm.getDistance();

        double speed = -(differencer / Math.abs(differencer));

        double rotatingArmSpeed = speed;

        MotorArm.set(ControlMode.PercentOutput, setpoint);

        final double feedForward = ArmController.rotatingArmFeedForward.calculate(rotatingArmSpeed);
        double output = ArmController.rotatingArmPIDController.calculate(getAngleArm(), setpoint);

        return (output * 0.1 + feedForward * 0.1) / 10;
    }

    /**
     * 0
     * 
     * @param magnitude set magnitude to a value of -1 to 1
     * @see ControlMode.PercentOutput
     */
    public void analogArm(double magnitude) {
        MotorArm.set(ControlMode.PercentOutput, magnitude);
    }

    /**
     * @param speed set speed to a value of -1 to 1
     * @see CANSparkMax.set()
     */
    public void analogCarriage(double speed) {
        MotorCarriage.set(speed);
    }

    /**
     * Calibrates the encoders when called. (RESETS VALUES TO 0) (USE WITH LIMIT
     * SWITCH)
     * 
     * @see Encoder
     */
    public void calibrate() {
        EncoderArm.reset();
        EncoderCarriage.reset();
    }

    public void putDashboard() {
        SmartDashboard.putNumber("EncoderArm", getAngleArm());
        SmartDashboard.putNumber("EncoderCarriage", getAngleCarriage());
        SmartDashboard.putNumber("Encoder Rat Arm", getRateArm());
        SmartDashboard.putNumber("Encoder Carriage Rate", getRateCarriage());
    }

}