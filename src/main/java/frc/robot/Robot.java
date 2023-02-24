// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

 * <p>
 * In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
    private static final int FRONT_LEFT_WHEEL_CAN_ID = 3; 
    private static final int BACK_LEFT_WHEEL_CAN_ID = 1; 
    private static final int FRONT_RIGHT_WHEEL_CAN_ID = 2; 
    private static final int BACK_RIGHT_WHEEL_CAN_ID = 4;
    
    private static final int ROTATING_ARM_CONTROLLER_CAN_ID = 6; 
    private static final int ROTATING_ARM_ENCODER_PIN_A = 0; // TODO: Set this properly once it's plugged in
    private static final int ROTATING_ARM_ENCODER_PIN_B = 1; // TODO: Set this properly once it's plugged in

    private static final int GRIPPER_CARRIAGE_CONTROLLER_CAN_ID = 9;
    private static final int GRIPPER_CARRIAGE_ENCODER_PIN_A = 0; // TODO
    private static final int GRIPPER_CARRIAGE_ENCODER_PIN_B = 1; // 
    
    private static final int XBOX_CONTROLLER_USB_PORT = 0; 
    private static final int BUTTON_PANEL_USB_PORT = 4;

    ADIS16470_IMU gyro = new ADIS16470_IMU();
    MotorController frontLeft = new SparkWrapper(FRONT_LEFT_WHEEL_CAN_ID);
    MotorController backLeft = new SparkWrapper(BACK_LEFT_WHEEL_CAN_ID);
    MotorController frontRight = new SparkWrapper(FRONT_RIGHT_WHEEL_CAN_ID);
    MotorController backRight = new SparkWrapper(BACK_RIGHT_WHEEL_CAN_ID);
    
    XboxController xboxController = new XboxController(XBOX_CONTROLLER_USB_PORT);
    GenericHID buttonPanel = new GenericHID(BUTTON_PANEL_USB_PORT);

    MecanumDrive drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    
    private Encoder rotatingArmEncoder;
    MotorController rotatingArmController = new SparkWrapper(ROTATING_ARM_CONTROLLER_CAN_ID);

    private Encoder gripperCarriageEncoder; 
    private MotorController gripperCarriageController = new SparkWrapper(GRIPPER_CARRIAGE_CONTROLLER_CAN_ID);

    // Use gyro declaration from above here

    // The gain for a simple P loop
    double kP = 1;
    // The heading of the robot when starting the motion
    double heading;
    double gyro_error = 0;

    // Initialize motor controllers and drive

    MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, backLeft);
    MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, backRight);


    // Gains are for example purposes only - must be determined for your own robot!
    // TODO: Update gain vals in the feed forward 
    private final SimpleMotorFeedforward rotatingArmFeedForward = new SimpleMotorFeedforward(1, 3);
    private final PIDController rotatingArmPIDController = new PIDController(1, 0, 0);

    private void initializeRotatingArmEncoder() {
        rotatingArmEncoder = new Encoder(ROTATING_ARM_ENCODER_PIN_A, 
                                        ROTATING_ARM_ENCODER_PIN_B, 
                                        false, 
                                        Encoder.EncodingType.k2X); // TODO: Check if this is the Encoding type we want
        // Use SetDistancePerPulse to set the multiplier for GetDistance
        // This is set up assuming a 6 inch wheel with a 360 CPRh encoder.
        rotatingArmEncoder.setDistancePerPulse((Math.PI * 6) / 360.0);

        rotatingArmEncoder.reset();
    }

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

        initializeRotatingArmEncoder();

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
        SmartDashboard.putNumber("Encoder", rotatingArmEncoder.getDistance());
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
        STRAFE = bufferJoystickInput(leftJoystickX, 0.2); 
        STRAFE *= -1;

        var FORWARD = 0.0;
        FORWARD = bufferJoystickInput(leftJoystickY, 0.2);
        // Joystick direction is opposite 
        FORWARD *= -1;

        var ROTATE = 0.0;
        // Buffer the input
        ROTATE = bufferJoystickInput(rightJoystickX, 0.2); 

        var multiplier = 1;// (-RightStick.getThrottle() * 0.5) + 0.5;
        FORWARD *= multiplier;
        STRAFE *= multiplier;
        ROTATE *= multiplier;
        drive.driveCartesian(FORWARD, STRAFE, ROTATE);

        if (xboxController.getXButton()) {
            // Right now, just use the X button to rotate the arm
            rotateArmToPosition(.45); // TODO: What's the right value here? 
        }

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

    }

    public void setRotatingArmSpeed(double rotatingArmSpeedMetersPerSecond) {
        final double feedForward = rotatingArmFeedForward.calculate(rotatingArmSpeedMetersPerSecond);
        
        final double output =
            rotatingArmPIDController.calculate(rotatingArmEncoder.getRate(), rotatingArmSpeedMetersPerSecond);
        rotatingArmController.setVoltage(output + feedForward); 
      }
    

    private void rotateArmToPosition(double distance) {
        // Read 
        // Turn rotatingArm motor controller on
        if (rotatingArmEncoder.getDistance() < distance) {
            setRotatingArmSpeed(0.2); // TODO: Make this speed a bit smarter
        }
        SmartDashboard.putNumber("Encoder Distance", rotatingArmEncoder.getDistance());
        SmartDashboard.putNumber("Encoder Rate", rotatingArmEncoder.getRate());

    }

    private void holdRotatingArmPosition() {

    }

}