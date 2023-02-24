// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

//liveshare link
//dj's emporkails
//ahabdmer@gmail.com, troubleteamleader@gmail.com, hbFallecy@protonmail.com, kimsoobachow@gmail.com, nathanhalerobotics@gmail.com
/**
 * ACTIVATE LIVE SHARE IMMEDIATELY AS YOU OPEN THE LAPTOP
 * ACTIVATE LIVE SHARE IMMEDIATELY AS YOU OPEN THE LAPTOP
 * ACTIVATE LIVE SHARE IMMEDIATELY AS YOU OPEN THE LAPTOP
 * ACTIVATE LIVE SHARE IMMEDIATELY AS YOU OPEN THE LAPTOP
 * ACTIVATE LIVE SHARE IMMEDIATELY AS YOU OPEN THE LAPTOP
 * ACTIVATE LIVE SHARE IMMEDIATELY AS YOU OPEN THE LAPTOP
 * OTHERWISE I CANT DO ANYTHING WHILE IM IN THE PHILLIPINES
 * 
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>
 * Joystick analog values range from -1 to 1 and motor controller inputs also
 * range from -1 to 1
 * making it easy to work together.
 *
 * <p>
 * In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
    ADIS16470_IMU gyro = new ADIS16470_IMU();
    MotorController frontLeft = new SparkWrapper(3);
    MotorController backLeft = new SparkWrapper(1);
    MotorController frontRight = new SparkWrapper(2);
    MotorController backRight = new SparkWrapper(4);
    MotorController exstender = new SparkWrapper(9);
    MotorController lifter = new SparkWrapper(6);
    XboxController xboxController = new XboxController(0);

    GenericHID buttonPanel = new GenericHID(4);
    MecanumDrive drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    private static final int kJoystickPort = 0;
    private static final int kEncoderPortA = 0;
    private static final int kEncoderPortB = 1;

    // private MotorController m_motor;
    // private Joystick m_joystick;
    private Encoder m_encoder;

    // Use gyro declaration from above here

    // The gain for a simple P loop
    double kP = 1;
    // The heading of the robot when starting the motion
    double heading;
    double gyro_error = 0;

    // Initialize motor controllers and drive

    MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, backLeft);
    MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, backRight);

    /**
     *
     */
    // DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    @Override
    public void autonomousInit() {

    }

    //
    @Override
    public void autonomousPeriodic() {

        // TODO here:
        // Calculate desired heading from the vision module
        // heading = 45;

        gyro_error = heading - gyro.getAngle();
        // Drives forward continuously at half speed, using the gyro to stabilize the
        // heading
        double x = 0; // TODO: Figure this from the kp * error
        double y = 0; // TODO: Figure this from kp * error
        double z = 0; // TODO: Figure this form kp * error
        drive.driveCartesian(x, y, z);
    }

    @Override
    public void robotInit() {
        // get the library for the ADIS working.
        // I cannot install it and its causing me issues.
        // public static final ADIS16448_IMU imu = new ADIS16448_IMU();
        // m_motor = new CANSparkMax(frontLeft,MotorType.kBrushless);

        m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
        // Use SetDistancePerPulse to set the multiplier for GetDistance
        // This is set up assuming a 6 inch wheel with a 360 CPRh encoder.
        m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);

        // Set setpoint to current heading at start of auto
        heading = gyro.getAngle();

        // Invert the right side motors.
        // You may need to change or remove this to match your robot.
        frontRight.setInverted(true);
        backRight.setInverted(true);

        System.out.println("Robot Inited");

    }

    /*
     * The RobotPeriodic function is called every control packet no matter the
     * robot mode.
     */
    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
        // System.out.println("Periodic");
    }

    private final Lock dataLock = new ReentrantLock();
    // offset and distance must both be accessed under a `data_lock` lock!
    private float offset;
    private float distance;

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

    @Override
    public void teleopPeriodic() {
        dataLock.lock();
        var offset = this.offset;
        var distance = this.distance;
        dataLock.unlock();
        double leftJoystickY = xboxController.getLeftY();

        double leftJoystickX = xboxController.getLeftX();

        double rightJoystickX = xboxController.getRightX();
        // int panelJoystickAngle = buttonPanel.getPOV();
        // boolean button1_pressed = buttonPanel.getRawButtonPressed(1);
        // boolean button2_pressed = buttonPanel.getRawButtonPressed(2);
        // boolean button3_pressed = buttonPanel.getRawButtonPressed(3);
        // boolean button4_pressed = buttonPanel.getRawButtonPressed(4);
        /// boolean button5_pressed = buttonPanel.getRawButtonPressed(5);
        // boolean button6_pressed = buttonPanel.getRawButtonPressed(6);
        // boolean button9_pressed = buttonPanel.getRawButtonPressed(9);
        // boolean switch_flicked = buttonPanel.getRawButtonPressed(10);

        // motor speed is -1 to 1
        // m_motor.set(1);
        var STRAFE = 0.0;
        // Buffer the input 
        if (Math.abs(leftJoystickX) > 0.2) {
            if (leftJoystickX < 0) {
                STRAFE = leftJoystickX + 0.2;
            } else {
                STRAFE = leftJoystickX - 0.2;
            }
        }
        // Joystick direction is opposite of where we want to go
        STRAFE *= -1;

        var FORWARD = 0.0;
        // Buffer the input
        if (Math.abs(leftJoystickY) > 0.2) {
            if (leftJoystickY < 0) {
                FORWARD = leftJoystickY + 0.2;
            } else {
                FORWARD = leftJoystickY - 0.2;
            }
        }
        // Joystick direction is opposite 
        FORWARD *= -1;

        var ROTATE = 0.0;
        // Buffer the input
        if (Math.abs(rightJoystickX) > 0.2) {
            if (rightJoystickX < 0) {
                ROTATE = rightJoystickX + 0.2;
            } else {
                ROTATE = rightJoystickX - 0.2;
            }
        }

        var multiplier = 1;// (-RightStick.getThrottle() * 0.5) + 0.5;
        FORWARD *= multiplier;
        STRAFE *= multiplier;
        ROTATE *= multiplier;
        // drive.driveCartesian(-y, -x, -z); // z);
        drive.driveCartesian(FORWARD, STRAFE, ROTATE);

        /*
         * // Thumb button
         * if (rightStick.getRawButton(2)) {
         * shoot.setVoltage(-14.0);
         * } else {
         * shoot.setVoltage(0.0);
         * }
         */
        // drive.close();

        System.out.println("FORWARD: " + FORWARD + " STRAFE: " + STRAFE + " ROTATE: " + ROTATE);

        // System.out.println("teleopPeriodic");
        // System.out.print(ROTATE);
        // System.out.print(FORWARD);
        // System.out.print(STRAFE);
    }

}