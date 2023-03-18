// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController; 
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DigitalInput; //damn!!! digital input, cant remember where I needed that
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.geometry.Translation2d;
//import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;

//liveshare link
/**
 * 
 * I blame neil for this existing. - DJ
 * 
 */

public class Robot extends TimedRobot {
    private static final int FRONT_LEFT_WHEEL_CAN_ID = 3;
    private static final int BACK_LEFT_WHEEL_CAN_ID = 1;
    private static final int FRONT_RIGHT_WHEEL_CAN_ID = 2;
    private static final int BACK_RIGHT_WHEEL_CAN_ID = 4;

    private static final int ROTATING_ARM_CONTROLLER_CAN_ID = 6; // still doesnt work \o/
    private static final int ROTATING_ARM_ENCODER_PIN_A = 3; // TODO: Set this properly once it's plugged in
    private static final int ROTATING_ARM_ENCODER_PIN_B = 4; // currently set up to be the gripper carriage encoder

    private static final int GRIPPER_CARRIAGE_CONTROLLER_CAN_ID = 9;
    private static final int GRIPPER_CARRIAGE_ENCODER_PIN_A = 1; //
    private static final int GRIPPER_CARRIAGE_ENCODER_PIN_B = 0; //
    /* Little explanation:
     * CANSparkMax is a class dedicated to specifically the Rev Robotics Sparkmax motor controller
     * Below we are defining the objects frontLeft etc. to their CAN ID (checked using the rev robotics hardware client)
     * and Motor Type (brushless or brushed usually)
     * Then with the Mecanum Drive class we plug in our CANSparkMax objects to initialize our object "drive".
     * This is opposed to the motorcontroller class we have, which is what we used last year and is combined with SparkWrapper. Didn't make it, blame neil.
     * 
     * The Relative Encoder class
    */
    CANSparkMax frontLeft = new CANSparkMax(FRONT_LEFT_WHEEL_CAN_ID, MotorType.kBrushless);
    CANSparkMax backLeft = new CANSparkMax(BACK_LEFT_WHEEL_CAN_ID, MotorType.kBrushless);
    CANSparkMax frontRight = new CANSparkMax(FRONT_RIGHT_WHEEL_CAN_ID, MotorType.kBrushless);
    CANSparkMax backRight = new CANSparkMax(BACK_RIGHT_WHEEL_CAN_ID, MotorType.kBrushless);
    CANSparkMax gripperCarriageController = new CANSparkMax(GRIPPER_CARRIAGE_CONTROLLER_CAN_ID,
            MotorType.kBrushed);
    CANSparkMax LiftAxisController = new CANSparkMax(ROTATING_ARM_CONTROLLER_CAN_ID, MotorType.kBrushless);
    MecanumDrive drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, backLeft);
    MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, backRight);
    private SparkMaxPIDController LiftAxisPID;
    private RelativeEncoder LiftAxisEncoder;
    private RelativeEncoder frontLeftEnc = frontLeft.getEncoder();
    private RelativeEncoder backLeftEnc = backLeft.getEncoder();
    private RelativeEncoder frontRightEnc = frontRight.getEncoder();
    private RelativeEncoder backRightEnc = backRight.getEncoder();
    private Encoder rotatingArmEncoder = new Encoder(ROTATING_ARM_ENCODER_PIN_A,
            ROTATING_ARM_ENCODER_PIN_B,
            false,
            Encoder.EncodingType.k2X);
    private Encoder gripperCarriageEncoder;
    ADIS16448_IMU gyro = new ADIS16448_IMU();
    Gyro aGyro = (Gyro)(gyro);
    //solenoids
    DoubleSolenoid Stage1Helper;
    DoubleSolenoid Stage2Gripper;
    Solenoid breakpiston;
    private static final int XBOX_CONTROLLER_USB_PORT = 0; // its hood habit for variables
    private static final int XBOX_CONTROLLER_USB_PORT2 = 1; // a little uneccessary tho
    private static final int BUTTON_PANEL_USB_PORT = 4;
    XboxController xboxController = new XboxController(XBOX_CONTROLLER_USB_PORT);
    XboxController xboxController2 = new XboxController(XBOX_CONTROLLER_USB_PORT2);
    GenericHID buttonPanel = new GenericHID(BUTTON_PANEL_USB_PORT);
    //Vision thread 
    Thread RRvisionThread;
    NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    NetworkTable visionNetworkTable = networkTables.getTable("Vision");
        // ^^^^^ We'll use this to read values from the Raspberry Pi vision component
    boolean autoswitcher = false; //for switching auto modes before auto
    boolean automode;   //actual switch variable
    int a = 0;
    double pos1;
    double sim;
    double rotations;
    double hardcodeddistance;
    // variables for auto
    double x = 0; // TODO: Figure this from the kp * error
    double y = 0; // TODO: Figure this from kp * error
    double z = 0; // TODO: Figure this form kp * error
    // The heading of the robot when starting the motion
    double heading;
    double gyro_error = 0;
    long t = System.currentTimeMillis();
    long end;
    private final Lock dataLock = new ReentrantLock();
    // TODO: Update gain vals in the feed forward
    private final SimpleMotorFeedforward rotatingArmFeedForward = new SimpleMotorFeedforward(1, 3);
    private final PIDController rotatingArmPIDController = new PIDController(1, 1, 0);
  
    @Override
    public void autonomousInit() {
        gyro.calibrate();
        gyro.reset();
        heading = gyro.getAngle(); //gets init angle
        automode = true;
        if (automode == true){
            end = System.currentTimeMillis() + 30000;
        } else {
            end = System.currentTimeMillis() + 30000;
        }
        
        hardcodeddistance = 0;
        autoswitcher = true;
    }
    
    @Override
    public void autonomousPeriodic() {
        // TODO here:
        // Calculate desired heading from the vision module
        heading = 0;
        gyro_error = heading - gyro.getAngle();
        if (System.currentTimeMillis() < end && automode == true) {
            gyrocorrect();
            double setpoint = -180.0;
            System.out.println(gyro.getGyroAngleX());
            drive.driveCartesian(x, y, z); // TODO: This is wrong
        } else if(System.currentTimeMillis() < end && automode == false) {
            System.out.print("a");
        } else {
            drive.driveCartesian(0, 0, 0);
        }
    }

    @Override
    public void robotInit() {
        //reset things
        rotatingArmEncoder.reset();
        gyro.calibrate();
        gyro.reset();
        // vision thread method
        visionthread();
        initSolenoid();
        frontRight.setInverted(true);
        backRight.setInverted(true);
        
        initializeRotatingArmEncoder();

        System.out.println("Robot Inited");

    }

    @Override
    public void robotPeriodic() {
        putGyroDataOnDashboard();
        if(xboxController.getBButton() && autoswitcher==false) {
            automode = !automode;
            System.out.println(automode);
        }
    }

    @Override
    public void teleopInit() {
        gyro.calibrate();
        gyro.reset();
        pos1=0;
        sim=0;
    }

    @Override
    public void teleopPeriodic() {
        dataLock.lock();
        dataLock.unlock();

        inputs();

    }

    public double integralfunction(double accel) {
        // yuh angus hours
        double positionoffset;
        long time = System.currentTimeMillis();

        positionoffset = ((1 / 2) * accel * time * time); // theoretically how physics work

        return (positionoffset);
    }

    public void gyrocorrect() {
        //please correct thineself sir!
        double setpoint = 7.0;
        System.out.println(gyro.getGyroAngleX());
            if (gyro.getGyroAngleX() > 0.2+setpoint) {
                 z = .1;
            } else if (gyro.getGyroAngleX() < -0.1) {
                 z = -0.1;
            } else {
                z = 0;
            }

            if (gyro.getGyroAngleY() > 0.2+setpoint) {
                 x = -0.1;
            } else if (gyro.getGyroAngleY() < -0.2) {
                 x = 0.1;
            } else {
                x = 0;
            }
        SmartDashboard.putNumber("new X: ", x);
        SmartDashboard.putNumber("new Y: ", y);
    }

    private void putGyroDataOnDashboard() {
        SmartDashboard.putNumber("Gyro Accel X", gyro.getAccelX());
        SmartDashboard.putNumber("Gyro Accel Y", gyro.getAccelY());
        SmartDashboard.putNumber("Gyro Accel Z", gyro.getAccelZ());

        SmartDashboard.putNumber("Gyro Angle X", gyro.getGyroAngleX());
        SmartDashboard.putNumber("Gyro Angle Y", gyro.getGyroAngleY());
        SmartDashboard.putNumber("Gyro Angle Z", gyro.getGyroAngleZ());

        SmartDashboard.putNumber("Gyro Angle: ", gyro.getAngle());

        SmartDashboard.putNumber("Gyro Rate ", gyro.getRate());
        SmartDashboard.putNumber("Encoder Distance", rotatingArmEncoder.getDistance());
        SmartDashboard.putNumber("Encoder Rate", rotatingArmEncoder.getRate());
        SmartDashboard.putNumber("Encoder Raw", rotatingArmEncoder.getRaw());
    }
    
    private void initializeRotatingArmEncoder() {
        // TODO: Check if this is the Encoding type we want
        // Use SetDistancePerPulse to set the multiplier for GetDistance
        // This is set up assuming a 6 inch wheel with a 360 CPRh encoder.
        double rotation = 360.0/2048.0;
        rotatingArmEncoder.setDistancePerPulse(rotation);
        rotatingArmEncoder.setSamplesToAverage(5);
        rotatingArmEncoder.setMinRate(0.05);

        rotatingArmEncoder.reset(); // GRAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH
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

    public void initSolenoid(){
        Stage1Helper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 5);
        Stage2Gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 6);
        Stage1Helper.set(kReverse); // real init hours
        breakpiston = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        breakpiston.set(true); // true on jah - DJ
    }
    public void inputs() {
        double leftJoystickY = xboxController.getLeftY();
        double leftJoystickX = xboxController.getLeftX();
        double rightJoystickX = xboxController.getRightX();
        // System.out.println(rotatingArmEncoder.getDistance()+" arm rotation angle
        // position ---------");
        // int panelJoystickAngle = buttonPanel.getPOV();
        // boolean button1_pressed = buttonPanel.getRawButtonPressed(1);
        // boolean button2_pressed = buttonPanel.getRawButtonPressed(2);
        // boolean button3_pressed = buttonPanel.getRawButtonPressed(3);
        // boolean button4_pressed = buttonPanel.getRawButtonPressed(4);
        // boolean button5_pressed = buttonPanel.getRawButtonPressed(5);
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

        var multiplier = (-(xboxController.getLeftTriggerAxis() * .3) + (xboxController.getRightTriggerAxis() * .4)
                + .5); // (-RightStick.getThrottle() * 0.5) + 0.5;

        FORWARD *= multiplier;
        STRAFE *= multiplier;
        ROTATE *= multiplier;
        drive.driveCartesian(FORWARD, -STRAFE, ROTATE);

        LiftAxisController.set(((xboxController2.getLeftTriggerAxis() - xboxController2.getRightTriggerAxis()) * .2));
        double c = 0; // shrimple and inefficient
        double d = 0;
        // if I give it some more voltage hrmrmm I can make it hold up but it will start
        // slipping
        if (xboxController2.getLeftBumper()) {
            gripperCarriageController.set(1.0);
            // System.out.println("carriage.");
            c = 1;
        } else if (xboxController2.getRightBumper()) {
            gripperCarriageController.set(-1.0);
            d = -1.0;
        } else {
            gripperCarriageController.set(0);
            c = 0;
            d = 0;
        }
        if (xboxController.getLeftBumper()) {
            System.out.print("You are an idiot. Stop it.");
        } else if (xboxController.getRightBumper()) {
            System.out.print("You are an idiot. Stop it.");
        }

        gripperCarriageController.set(.05 + c + d); // TODO: re-enable when the window motor is fixed
        // theoretically this would let it fight gravity while responding to my
        // controls. Not tuned yet.

        if (xboxController2.getAButtonPressed()) {
            a += 1;
        }

        if (a % 2 == 0) {
            Stage2Gripper.set(kForward); // toggle switch
        } else {
            Stage2Gripper.set(kReverse);
        }

        if (xboxController2.getYButtonPressed()) {
            Stage1Helper.toggle();
        }

        if (xboxController.getBButtonPressed()) {
            pos1 = 0;
            breakpiston.toggle();

        }
        if (xboxController2.getXButton()) {
            sim = 1.0;

            SmartDashboard.putNumber("pos1", pos1);

        } else {
            sim = 0.0;

            // If X is not pressed, stop the arm from moving
        }
    }
    public void visionthread(){
        RRvisionThread = new Thread(
                () -> {
                    // Get the UsbCamera from CameraServer
                    UsbCamera camera = CameraServer.startAutomaticCapture();
                    // Set the resolution
                    camera.setResolution(640, 480);

                    // Get a CvSink. This will capture Mats from the camera
                    CvSink cvSink = CameraServer.getVideo();
                    // Setup a CvSource. This will send images back to the Dashboard
                    CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

                    // Mats are very memory expensive. Lets reuse this Mat.
                    Mat mat = new Mat();

                    // This cannot be 'true'. The program will never exit if it is. This
                    // lets the robot stop this thread when restarting robot code or
                    // deploying.
                    while (!Thread.interrupted()) {
                        // Tell the CvSink to grab a frame from the camera and put it
                        // in the source mat. If there is an error notify the output.
                        if (cvSink.grabFrame(mat) == 0) {
                            // Send the output the error.
                            // outputStream.notifyError(cvSink.getError());
                            // skip the rest of the current

                            continue;
                        }
                        // Put a rectangle on the image
                        Imgproc.rectangle(
                                mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                        // Give the output stream a new image to display
                        outputStream.putFrame(mat);
                    }
                });
        RRvisionThread.setDaemon(true); // haha daemon!
        RRvisionThread.start();
    }
    /* ALL THINGS PATHPLANNER BELOW */
                // Locations of the wheels relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
    //Rotation2d m_gyro = new  
    // Creating my odometry object from the kinematics object and the initial wheel positions.
    MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
        m_kinematics,
        aGyro.getRotation2d(),
        new MecanumDriveWheelPositions(
        frontLeftEnc.getPosition(), frontRightEnc.getPosition(),
        backLeftEnc.getPosition(), backRightEnc.getPosition()
        ),
        new Pose2d(5.0, 13.5, new Rotation2d())
    );
    
}
