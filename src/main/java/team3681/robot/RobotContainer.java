package team3681.robot;

import team3681.robot.lib.drivebase.MDrive;
import team3681.robot.lib.hardware.motor.interfaces.UniversalMotor;
import team3681.robot.lib.hardware.motor.model.SparkWrapper;
import team3681.robot.lib.hardware.motor.model.VictorWrapper;
import team3681.robot.lib.hardware.HID.HIDWrapper;
import team3681.robot.lib.hardware.HID.HIDWrapperBuilder;
import team3681.robot.subsystem.State.Events.AutoManager;
import team3681.robot.subsystem.State.Events.TeleopManager;
import team3681.robot.subsystem.State.Events.Command.CommandHandler;
import team3681.robot.subsystem.State.Events.interfaces.BehaviorManager;
import team3681.robot.subsystem.State.arm.ArmController;
import team3681.robot.subsystem.State.arm.ArmState;
import team3681.robot.subsystem.State.arm.ArmWrapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPlanner;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
// import com.pathplanner.lib.PathPoint;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The main class to put all the functions.
 * 
 * @note The methods called here will be used in Robot.Java
 * @see Robot
 * 
 */
public class RobotContainer {

    // NOTE: ID Constants, DO NOT CHANGE UNLESS YOU KNOW WHAT YOU ARE DOING!
    private static final int FRONT_LEFT_WHEEL_CAN_ID = 3;
    private static final int BACK_LEFT_WHEEL_CAN_ID = 1;
    private static final int FRONT_RIGHT_WHEEL_CAN_ID = 2;
    private static final int BACK_RIGHT_WHEEL_CAN_ID = 4;

    private static final int SPINNER_A = 16;
    private static final int SPINNER_B = 17;

    private static final int ARM_CONTROLLER_CAN_ID = 6;
    private static final int ARM_ENCODER_PIN_A = 0;
    private static final int ARM_ENCODER_PIN_B = 1;

    private static final int CARRIAGE_CONTROLLER_CAN_ID = 9;
    private static final int CARRIAGE_ENCODER_PIN_A = 2;
    private static final int CARRIAGE_ENCODER_PIN_B = 3;

    private static final int CONTROLLER_USB_PORT_A = 0;
    private static final int CONTROLLER_USB_PORT_B = 1;

    private static final int BUTTON_PANEL_USB_PORT = 2;

    private static final int LIMIT_SWITCH_PORT_A = 4;
    private static final int LIMIT_SWITCH_PORT_B = 5;

    private static final int FORWARD_SOLENOID_PORT_A = 7;
    private static final int REVERSE_SOLENOID_PORT_A = 5;
    private static final int FORWARD_SOLENOID_PORT_B = 4;
    private static final int REVERSE_SOLENOID_PORT_B = 6;

    // NOTE: Regular constants
    private static final double INPUT_BUFFER_AMOUNT = 0.2;

    // NOTE: I/O
    // private static final XboxController controllerA = new XboxController(CONTROLLER_USB_PORT_A);
    // private static final XboxController controllerB = new XboxController(CONTROLLER_USB_PORT_B);
    // private static final GenericHID buttonPanel = new GenericHID(BUTTON_PANEL_USB_PORT);
    
    private static final HIDWrapper controllerA = new HIDWrapperBuilder()
        .withXboxController(CONTROLLER_USB_PORT_A)
        .build();

    private static final HIDWrapper controllerB = new HIDWrapperBuilder()
        .withXboxController(CONTROLLER_USB_PORT_B)
        .build();
    
    private static final HIDWrapper buttonPanel = new HIDWrapperBuilder()
        .withGenericHID(BUTTON_PANEL_USB_PORT)
        .build();
    
    private static final BehaviorManager TOP = new TeleopManager();
    private static final BehaviorManager AUTO = new AutoManager();

    // NOTE: Motors
    //TODO: Create builders for motors.
    /* example:
     * 
     * new MotorBuilder;
     * .with(MotorType)
     * .brushed/brushless
     * .port(#)
     * .name("")
     * .build();
     */
    // private static final UniversalMotor frontLeft = new SparkWrapper(FRONT_LEFT_WHEEL_CAN_ID, "Front Left", true);
    // private static final UniversalMotor backLeft = new SparkWrapper(BACK_LEFT_WHEEL_CAN_ID, "Back Left", true);
    // private static final UniversalMotor frontRight = new SparkWrapper(FRONT_RIGHT_WHEEL_CAN_ID, "Front Right", true);
    // private static final UniversalMotor backRight = new SparkWrapper(BACK_RIGHT_WHEEL_CAN_ID, "Back Right", true);

    // private static final UniversalMotor spinnerA = new VictorWrapper(SPINNER_A, "Spinner A");
    // private static final UniversalMotor spinnerB = new VictorWrapper(SPINNER_B, "Spinner B");

    // private static final UniversalMotor armMotor = new SparkWrapper(ARM_CONTROLLER_CAN_ID, "Rotating Arm", true);
    // private static final UniversalMotor carriageMotor = new SparkWrapper(CARRIAGE_CONTROLLER_CAN_ID, "Carriage", false);

    private static final UniversalMotor frontLeft = new SparkWrapper.Builder(FRONT_LEFT_WHEEL_CAN_ID)
        .withMotorName("Front Left")
        .withMotorType(MotorType.kBrushless)
        .withIdleMode(IdleMode.kBrake)
        .build();

    // NOTE: Encoders
    private static final Encoder armEncoder = new Encoder(ARM_ENCODER_PIN_A, ARM_ENCODER_PIN_B, false,
            CounterBase.EncodingType.k4X);
    private static final Encoder carriageEncoder = new Encoder(CARRIAGE_ENCODER_PIN_A, CARRIAGE_ENCODER_PIN_B, false,
            CounterBase.EncodingType.k4X);

    // NOTE: Solenoids
    DoubleSolenoid armPistonSolenoid;
    DoubleSolenoid handPistonSolenoid;
    Solenoid brakePistonSolenoid;

    // NOTE: Misc Digital Input
    DigitalInput LSA = new DigitalInput(LIMIT_SWITCH_PORT_A);
    DigitalInput LSB = new DigitalInput(LIMIT_SWITCH_PORT_B);

    // NOTE: Drives / Actors
    //public static final ArmWrapper MainArm = new ArmWrapper(armEncoder, carriageEncoder, armMotor, carriageMotor, spinnerA, spinnerB);
    //public static final MDrive drive = new MDrive(frontLeft, backLeft, frontRight, backRight);
    public static final ArmController armController = new ArmController(MainArm, ArmState.RecalibrateWait);
    public static final ADIS16448_IMU gyro = new ADIS16448_IMU();

    // NOTE: Raspberry PI interface related variables
    // TODO: Read values from the Raspberry Pi vision component
    NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    NetworkTable visionNetworkTable = networkTables.getTable("Vision");

    // NOTE: Variables for auto
    double x = 0.0; // TODO: Figure this from the kp * error
    double y = 0.0; // TODO: Figure this from kp * error
    double z = 0.0; // TODO: Figure this form kp * error

    Timer rollerTimer = new Timer("Roller Timer");

    // NOTE: CONSTANTS ALL GO HERE
    public void onStart() {
        initSolenoid(FORWARD_SOLENOID_PORT_A, REVERSE_SOLENOID_PORT_A,
            FORWARD_SOLENOID_PORT_B, REVERSE_SOLENOID_PORT_B);
    }

    public void teleop() {
        
    }

    public void autonomousMode() {

    }

    public void onDisable() {

    }

    /**
     * As of 2023, smartdashboard is the way to go for drivers to see on the field. I strongly suggest SmartDashboard over ShuffleBoard.
     * Put all of the information you want to see, or your drivers to see on the screen. Put all the information you would want as a driver on the board.
     * Tell your drivers what the funky numbers mean.
     * @see SmartDashboard
     */
    public void putDashboard() {
        SmartDashboard.putNumber("Gyro Accel X", gyro.getAccelX());
        SmartDashboard.putNumber("Gyro Accel Y", gyro.getAccelY());
        SmartDashboard.putNumber("Gyro Accel Z", gyro.getAccelZ());

        SmartDashboard.putNumber("Gyro Angle X", gyro.getGyroAngleX());
        SmartDashboard.putNumber("Gyro Angle Y", gyro.getGyroAngleY());
        SmartDashboard.putNumber("Gyro Angle Z", gyro.getGyroAngleZ());

        SmartDashboard.putNumber("Gyro Angle: ", gyro.getAngle());

        SmartDashboard.putNumber("Gyro Rate ", gyro.getRate());

        SmartDashboard.putBoolean("Limit Switch A State", LSA.get());
        SmartDashboard.putBoolean("Limit Switch B State", LSB.get());

        SmartDashboard.putNumber("ROLLER MODE CHANGE PISTONS", armPistonSolenoid.getFwdChannel());

        //NOTE: Methods from other objects to put their information on display. It's kinda dumb. Implement something smarter if you want, im not gonna do it.
        MainArm.putDashboard();
        
        armController.putDashboard();
    }

    /**
     * Made for the 2023 season, not important. This initializess three solenoid objects, two double &  one single and sets whatever is needed to default state.
     * @see DoubleSolenoid
     * @see Solenoid
     */
    private void initSolenoid(int fc1, int rc1, int fc2, int rc2) {
        armPistonSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, fc1, rc1); // NOTE: Forward channel, reverse
        handPistonSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, fc2, rc2);
        armPistonSolenoid.set(kReverse);
        brakePistonSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        brakePistonSolenoid.set(true);
    }

    private double bufferJoystickInput(double inputValue, double bufferAmt) {
        if (Math.abs(inputValue) > bufferAmt) {
            if (inputValue < 0) {
                return inputValue + bufferAmt;
            } else {
                return inputValue - bufferAmt;
            }
        }
        return 0.0;
    }
}