package arm;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix.motorcontrol.ControlMode;

import lib.interfaces.MotorInterface;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class ArmWrapper {

    private Encoder EncoderArm;
    private Encoder EncoderCarriage;
    private MotorInterface MotorArm;
    private CANSparkMax MotorCarriage;

    static final SimpleMotorFeedforward rotatingArmFeedForward = new SimpleMotorFeedforward(0.38123, 0.07469);
    static final PIDController rotatingArmPIDController = new PIDController(1.7, 0.0,0.25); //2.0565, 0.0, 0.10);

    static final SimpleMotorFeedforward carriageFeedForward = new SimpleMotorFeedforward(1.422, 0.54615);
    static final PIDController carriagePIDController = new PIDController(22.616, 0.0, 2.2913);

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
        double differencer = getAngleArm()-setpoint;

        double speed = -(differencer / Math.abs(differencer));

        final double feedForward = rotatingArmFeedForward.calculate(speed);
        double output = rotatingArmPIDController.calculate(getAngleArm(), setpoint);

        return ((output*0.1)+(feedForward*0.1))/10;
    }

    public double getPIDoutG(double setpoint) {
        double differencer = getAngleCarriage()-setpoint;

        double speed = -(differencer / Math.abs(differencer));

        final double feedForward = carriageFeedForward.calculate(speed);
        double output = carriagePIDController.calculate(getAngleCarriage(), setpoint);

        return ((output*0.1)+(feedForward*0.1)/10);
    }
    /**
     * Only reason for this to exist is to satiate my needs to differentiate it from the analog control
     * @param setpoint desired angle
     */
    public void PIDControlArm(double setpoint, boolean safety){
        if (safety) {
        MotorArm.set2(getPIDout(setpoint));
        }
        SmartDashboard.putNumber("Chosen Setpoint", setpoint);
        SmartDashboard.putNumber("PID OUT", getPIDout(setpoint));
        SmartDashboard.putNumber("Differencer", getAngleArm()-setpoint);

    }

    public void PIDControlCarriage(double setpoint, boolean safety){
        if (safety) {
        MotorCarriage.set(getPIDoutG(setpoint));
        }
        SmartDashboard.putNumber("Chosen Setpoint", setpoint);
        SmartDashboard.putNumber("PID OUT", getPIDoutG(setpoint));
        SmartDashboard.putNumber("Differencer", getAngleCarriage()-setpoint);

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

        double rA = 360.0/2048.0;
        EncoderArm.setDistancePerPulse(rA);
        EncoderArm.setSamplesToAverage(5);
        EncoderArm.setMinRate(0.05); 

        double rG = .0798;
        EncoderCarriage.setDistancePerPulse(rG);
        EncoderCarriage.setSamplesToAverage(5);
        EncoderCarriage.setMinRate(0.05);
        MotorCarriage.setIdleMode(IdleMode.kBrake);

    }
    /**
     * Drops ArmWrapper data onto the Smart Dashboard
     * @see SmartDashboard
     */
    public void putDashboard() {
        SmartDashboard.putNumber("EncoderArm", getAngleArm());
        SmartDashboard.putNumber("EncoderCarriage", getAngleCarriage());
        SmartDashboard.putNumber("Encoder Arm Rate", getRateArm());
        SmartDashboard.putNumber("Encoder Carriage Rate", getRateCarriage());
    }

}