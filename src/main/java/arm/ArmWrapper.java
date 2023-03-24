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
    private static double EPSILON = 3.0;

    private Encoder armEncoder;
    private Encoder carriageEncoder;
    private MotorInterface armMotor;
    private CANSparkMax carriageMotor;

    private MotorInterface spinnerA;
    private MotorInterface spinnerB;

    static final SimpleMotorFeedforward rotatingArmFeedForward = new SimpleMotorFeedforward(0.38123, 0.07469);
    static final PIDController rotatingArmPIDController = new PIDController(2.0, 0.0, 0.10);// 1.7, 0.0,0.25); //2.0565,
                                                                                            // 0.0, 0.10);

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
    public ArmWrapper(Encoder armEncoder, Encoder carriageEncoder, MotorInterface armMotor, CANSparkMax carriageMotor,
            MotorInterface spinnerA, MotorInterface spinnerB) {
        this.armEncoder = armEncoder;
        this.carriageEncoder = carriageEncoder;
        this.armMotor = armMotor;
        this.carriageMotor = carriageMotor;
        this.spinnerA = spinnerA;
        this.spinnerB = spinnerB;
    }

    /**
     * @return Gets the encoder-found angle on the rotation axis for the big arm
     */
    public double getArmAngle() {
        return armEncoder.getDistance();
    }

    /**
     * @return Gets the encoder-found angle on the rotation axis for the carriage
     */
    public double getCarriageAngle() {
        return carriageEncoder.getDistance();
    }

    /**
     * @return Gets the encoder-found rate on the rotation axis for the arm
     */
    public double getArmRate() {
        return armEncoder.getRate();
    }

    /**
     * @return Gets the encoder-found rate on the rotation axis for the carriage
     */
    public double getCarriageRate() {
        return carriageEncoder.getRate();
    }

    /**
     * @return the output for the PID's
     */
    public double getPIDout(double desiredAngle) {
        double differencer = getArmAngle() - desiredAngle;

        double speed = -(differencer / Math.abs(differencer));

        final double feedForward = rotatingArmFeedForward.calculate(speed);
        double output = rotatingArmPIDController.calculate(getArmAngle(), desiredAngle);

        return ((output * 0.1) + (feedForward * 0.1)) / 10;
    }

    public double getPIDoutG(double desiredAngle) {
        double differencer = getCarriageAngle() - desiredAngle;

        double speed = -(differencer / Math.abs(differencer));

        final double feedForward = carriageFeedForward.calculate(speed);
        double output = carriagePIDController.calculate(getCarriageAngle(), desiredAngle);

        return ((output * 0.1) + (feedForward * 0.1) / 10.0);
    }

    /**
     * Only reason for this to exist is to satiate my needs to differentiate it from
     * the analog control
     * 
     * @param desiredAngle
     */
    public boolean PIDControlArm(double desiredAngle) {
        double difference = getArmAngle() - desiredAngle;
        if (Math.abs(difference) <= EPSILON) {
            return true;
        }
            
        armMotor.set2(getPIDout(desiredAngle));

        SmartDashboard.putNumber("Chosen Setpoint ARM", desiredAngle);
        SmartDashboard.putNumber("PID OUT ARM", getPIDout(desiredAngle));
        SmartDashboard.putNumber("Differencer", getArmAngle() - desiredAngle);

        return false;
    }

    public boolean PIDControlCarriage(double desiredAngle) {
        double difference = getArmAngle() - desiredAngle;
        if (Math.abs(difference) <= EPSILON) {
            return true;
        }

        // NOTE: 0.0001 is added to ensure the carriage is kept up opposing gravity
        carriageMotor.set(getPIDoutG(desiredAngle) + 0.0001);

        SmartDashboard.putNumber("Chosen Setpoint CARRIAGE", desiredAngle);
        SmartDashboard.putNumber("PID OUT CARRIAGE", getPIDoutG(desiredAngle));
        SmartDashboard.putNumber("Differencer", getCarriageAngle() - desiredAngle);
        
        return false;
    }

    /**
     * 0
     * 
     * @param magnitude set magnitude to a value of -1 to 1
     * @see ControlMode.PercentOutput
     */
    public void analogArm(double magnitude) {
        armMotor.set(ControlMode.PercentOutput, magnitude);
    }

    /**
     * @param speed set speed to a value of -1 to 1
     * @see CANSparkMax.set()
     */
    public void analogCarriage(double speed) {
        carriageMotor.set(speed);
    }

    /**
     * Calibrates the encoders when called. (RESETS VALUES TO 0) (USE WITH LIMIT
     * SWITCH)
     * 
     * @see Encoder
     */
    public void calibrateArm() {
        armEncoder.reset();

        double rA = 360.0 / 2048.0;
        armEncoder.setDistancePerPulse(rA);
        armEncoder.setSamplesToAverage(5);
        armEncoder.setMinRate(0.05);
    }

    /**
     * Calibrates the encoders when called. (RESETS VALUES TO 0) (USE WITH LIMIT
     * SWITCH)
     * 
     * @see Encoder
     */
    public void calibrateCarriage() {
        carriageEncoder.reset();

        double rG = 0.0798;
        carriageEncoder.setDistancePerPulse(rG);
        carriageEncoder.setSamplesToAverage(5);
        carriageEncoder.setMinRate(0.05);
        carriageMotor.setIdleMode(IdleMode.kBrake);

    }

    public void spinOut() {
        spinnerA.set(ControlMode.PercentOutput, 0.8);
        spinnerB.set(ControlMode.PercentOutput, 0.8);
    }

    public void spinIn() {
        spinnerA.set(ControlMode.PercentOutput, -0.8);
        spinnerB.set(ControlMode.PercentOutput, -0.8);
    }

    /**
     * Drops ArmWrapper data onto the Smart Dashboard
     * 
     * @see SmartDashboard
     */
    public void putDashboard() {
        SmartDashboard.putNumber("EncoderArm", getArmAngle());
        SmartDashboard.putNumber("EncoderCarriage", getCarriageAngle());
        SmartDashboard.putNumber("Encoder Arm Rate", getArmRate());
        SmartDashboard.putNumber("Encoder Carriage Rate", getCarriageRate());
    }

}