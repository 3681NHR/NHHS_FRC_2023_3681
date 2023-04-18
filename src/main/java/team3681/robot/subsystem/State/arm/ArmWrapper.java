package team3681.robot.subsystem.State.arm;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3681.robot.lib.hardware.motor.interfaces.UniversalMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Class exclusively made for the 2023 year.
 */
public class ArmWrapper {
    private static double EPSILON = 3.0;

    private Encoder armEncoder;
    private Encoder carriageEncoder;
    private UniversalMotor armMotor;
    private UniversalMotor carriageMotor;

    private UniversalMotor spinnerA;
    private UniversalMotor spinnerB;

    static final SimpleMotorFeedforward rotatingArmFeedForward = new SimpleMotorFeedforward(0.38123, 0.07469);
    static final PIDController rotatingArmPIDController = new PIDController(2.0, 0.0, 0.10);
    // 1.7, 0.0,0.25); 2.0565, 0.0, 0.10);
    static final SimpleMotorFeedforward carriageFeedForward = new SimpleMotorFeedforward(1.422, 0.54615);
    static final PIDController carriagePIDController = new PIDController(10.616, 0.0, 2.2913);

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
    public ArmWrapper(Encoder armEncoder, Encoder carriageEncoder, UniversalMotor armMotor, UniversalMotor carriageMotor,
            UniversalMotor spinnerA, UniversalMotor spinnerB) {
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

    public void terminateArm(){
        armMotor.setVoltage(0);
        armMotor.set(ControlMode.PercentOutput, 0);
        armMotor.setVolt(0);
    }

    public void terminateCarriage(){
        carriageMotor.setVoltage(0.1);
    }

    public double getExtension(ArmState State){
        double FIN_SETPOINT;

        if (State == ArmState.Low){
            FIN_SETPOINT = -16;

            return FIN_SETPOINT;
        } else if (State == ArmState.Medium) {
            FIN_SETPOINT = -14;

            return FIN_SETPOINT;
        } else if (State == ArmState.High) {
            FIN_SETPOINT = -22;

            return FIN_SETPOINT;
        } else {
            return 0;
        }
    }

    /**
     * @return the output for the PID's
     */
    public double getPIDout(double desiredAngle) { //
        double differencer = getArmAngle() - desiredAngle; 

        double speed = -(differencer / Math.abs(differencer));

        final double feedForward = rotatingArmFeedForward.calculate(speed);
        double output = rotatingArmPIDController.calculate(getArmAngle(), desiredAngle);

        return ((output * 0.1) + (feedForward * 0.1)) / 10;
    }

    /**
     * @return the output for the PID's
     */
    public double getPIDoutG(double desiredAngle) {
        double differencer = getCarriageAngle() - desiredAngle;

        double speed = -(differencer / Math.abs(differencer));

        final double feedForward = carriageFeedForward.calculate(speed);
        double output = carriagePIDController.calculate(getCarriageAngle(), desiredAngle);

        return ((output * 0.1) + (feedForward * 0.1) / 10.0);
    }

    /**
     * Uses getPIDout readings to control the movement of the motor
     * 
     * @param desiredAngle
     */

     // robpt.java PIDControlArm(-60)
    public boolean PIDControlArm(double desiredAngle) { //
        double difference = getArmAngle() - desiredAngle;
        armMotor.setVolt(getPIDout(desiredAngle));

        if (Math.abs(difference) <= EPSILON) {
            return true;
        }
        SmartDashboard.putNumber("Chosen Setpoint ARM", desiredAngle);
        SmartDashboard.putNumber("PID OUT ARM", getPIDout(desiredAngle));
        SmartDashboard.putNumber("Differencer A", getArmAngle() - desiredAngle);

        return false;
    }

    public boolean PIDControlCarriage(double desiredAngle) {
        carriageMotor.setVolt(getPIDoutG(desiredAngle));

        // NOTE: 0.0001 is added to ensure the carriage is kept up opposing gravity

        SmartDashboard.putNumber("Chosen Setpoint CARRIAGE", desiredAngle);
        SmartDashboard.putNumber("PID OUT CARRIAGE", getPIDoutG(desiredAngle));
        SmartDashboard.putNumber("Differencer C", getCarriageAngle() - desiredAngle);

        return true;
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
        carriageMotor.setVolt(speed);
    }

    /**
     * Calibrates the encoders when called. (RESETS VALUES TO 0) (USE WITH LIMIT
     * SWITCH)
     * 
     * @see Encoder
     */
    public void calibrateArm() {
        armEncoder.reset();;
    }

    /**
     * Calibrates the encoders when called. (RESETS VALUES TO 0) (USE WITH LIMIT
     * SWITCH)
     * 
     * @see Encoder
     */
    public void calibrateCarriage() {
        carriageEncoder.reset();
    }

    public void generalCalibration(){
        double rG = 0.0798;
        carriageEncoder.setDistancePerPulse(rG);
        carriageEncoder.setSamplesToAverage(5);
        carriageEncoder.setMinRate(0.05);

        double rA = 360.0 / 2048.0;
        armEncoder.setDistancePerPulse(rA);
        armEncoder.setSamplesToAverage(5);
        armEncoder.setMinRate(0.05);
    }

    private final static double ABRITRARY_NUMBER = 0.8;

    public void spinOut() {
        spinnerA.set(ControlMode.PercentOutput, ABRITRARY_NUMBER);
        spinnerB.set(ControlMode.PercentOutput, ABRITRARY_NUMBER);
    }

    public void spinIn() {
        spinnerA.set(ControlMode.PercentOutput, -ABRITRARY_NUMBER);
        spinnerB.set(ControlMode.PercentOutput, -ABRITRARY_NUMBER);
    }

    public void spinStop(){
        spinnerA.set(ControlMode.PercentOutput, 0);
        spinnerB.set(ControlMode.PercentOutput, 0);
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