// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController; 
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput; // I found out what I was using it for
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.drive;
import lib.interfaces.motorinterface;
import lib.motor.SparkWrapper;
import lib.motor.VictorWrapper;
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

//import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.geometry.Translation2d;
//import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.CounterBase;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.pathplanner.lib.PathPoint;

import autoaction.armaction;

import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;
/**
 * Originally created by DJ (DANIEL JAYLEN) during the 2023 season for FRC
 * for the 3681 team.
 * 
 * This contains all the code for that season. Use it well, i've left plenty of comments.
 * @see TimedRobot
 */

public class Robot extends TimedRobot {
    /* Little explanation:
     * SparkWrapper is a wrapper class dedicated to specifically the Rev Robotics Sparkmax motor controller
     * Below we are defining the objects frontLeft etc. to their CAN ID (checked using the rev robotics hardware client)
     * and Motor Type (brushless or brushed usually)
     * Then with the Mecanum Drive class we plug in our SparkWrapper objects to initialize our object "drive".
     * 
     * Edit: Use the wrappers instead. Sparkwrapper and Victorwrapper with the motorinterface interface.
     * 
     * The Relative Encoder class is a Rev Robotics specific encoder class used for the integrated encoders on the NEO brushless motors. (Behaves similarly but with less setup)
     * The Encoder class is usually used for non-integrated encoders like our AMT103 Quadrature Encoder (which was a pain to deal with for weeks and weeks and weeks SCREAMING)
     * ADIS16448 is our gyro model
     * FRC is quite compatible with xbox controllers and are fairly straightforward.
     * Solenoids are pneumatic control valve things and are straightforward (this is a cue to look up wpilib documentation)
     * Network Tables is the key to transmitting data over from a R-Pi or jetson and the like
     * GenericHID is for our custom state of the art button panel!!! (real and based)
     * no idea what datalock was for
     * SimplePID and Feedforward was for PID controllers (look them up, its a controller not the disease thing redo your search)
     * Okay anything beyond this, literally hover over the funny looking word and it will come up with a definition or what values to put in it
     * 
     * Important notes:
     * (!)AVOID USING WHILE LOOPS, THE CODE IS ALWAYS RUNNING ANYWAY
     *     - USING AN UNTESTED WHILE LOOP, WE WERE DISABLED FOR 4 QUAL ROUNDS DURING THE 3/4/23 FRC GLACIER PEAK EVENT
     * (!) CHECK CONTINUITY FOR WIRES. IT ISNT ALWAYS YOUR FAULT
     *     - If not continuity, use an oscilloscope to test for data for sensors
     * The MXI port(the middle one) on the roborio is used for gyros and the like and it plugs in to all the pins.
     * (?) If someone who is better than me (likely) comes around these later years, check out other teams code.
     *      - This code isn't very good. Many other teams have had years of foundation building on their code. As of 2023, I'd like future team members
     *      - to do similar. Please post everything you do in github, and maybe when I come back in 5 years, everything will be awesome.
     *      - Check out other teams code on github, especially team 2910 for command-based programming and team 254 for timed-based similar to ours.
     *          - edit: I have learned so much after looking at other code. My horizons are broadened.
     * 
    */
    private static final int FRONT_LEFT_WHEEL_CAN_ID = 3;
    private static final int BACK_LEFT_WHEEL_CAN_ID = 1;
    private static final int FRONT_RIGHT_WHEEL_CAN_ID = 2;
    private static final int BACK_RIGHT_WHEEL_CAN_ID = 4;    
    
    armaction action = new armaction();
    motorinterface frontLeft = new SparkWrapper(FRONT_LEFT_WHEEL_CAN_ID, "frontLeft");
    motorinterface backLeft = new SparkWrapper(BACK_LEFT_WHEEL_CAN_ID, "Back Left");
    motorinterface frontRight = new SparkWrapper(FRONT_RIGHT_WHEEL_CAN_ID, "Front Right");
    motorinterface backRight = new SparkWrapper(BACK_RIGHT_WHEEL_CAN_ID, "Back Right");
    
    motorinterface spinner = new VictorWrapper(16, "spinna real");
    motorinterface spinner2 = new VictorWrapper(17, "real shit"); //I understand everything now 3/21/23
    
    //armaction action = new armaction();
    //double starttime = action.getTime()
    drive drive = new drive(frontLeft, backLeft, frontRight, backRight); //vrooo
    ADIS16448_IMU gyro = new ADIS16448_IMU();
    //Gyro aGyro = (Gyro)(gyro); BIG  NO CANNOT CAST
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
    double pos1;
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
    double fin = -70;

    private final Lock dataLock = new ReentrantLock();
    // TODO: Update gain vals in the feed forward

    private static volatile boolean wPressed = false;
    public static boolean isWPressed() {
        synchronized (Robot.class) {
            return wPressed;
        }
    }

    @Override
    public void disabledInit(){
        Stage1Helper.set(kReverse);
    }
    @Override
    public void robotInit() {
        //reset things
        action.initializeRotatingArmEncoder();
        action.calibrate();
        gyro.calibrate();
        gyro.reset();
        // vision thread method
        visionthread();
        initSolenoid(7, 5, 4, 6); //channels for solenoids
        

        System.out.println("Robot Inited");

    }
    @Override
    public void robotPeriodic() {
        putDashboard();
        if(Robot.isWPressed()) {
            automode = !automode;
        }
    }

    @Override
    public void teleopInit() {
        gyro.calibrate();
        gyro.reset();
    }

    @Override
    public void teleopPeriodic() {
        dataLock.lock();
        dataLock.unlock();
        inputs();
    }

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
        
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));
        PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);

        // Print the velocity at the sampled time

        
        //for impromptu, theoretically use this with the funny computere vision
        PathPlannerTrajectory traj1 = PathPlanner.generatePath(
            new PathConstraints(4, 3), 
            new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)), // position, heading
            new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45)) // position, heading
                
        );
    }

    @Override
    public void autonomousPeriodic() {
        // TODO here:
        // Calculate desired heading from the vision module
        heading = 0;
        gyro_error = heading - gyro.getAngle();
        if (System.currentTimeMillis() < end) {
            autonMode(automode);
        } else {
            drive.driveCartesian(0, 0, 0);
        }
    }
    // ========(EVERYTHING BELOW IS NOT PART OF TIMED ROBOT FRC STUFF)================================================
    public boolean emergencysend() {
        if (buttonPanel.getRawButton(1)) {
        return true;
        } else
        return false;
    }
    private void putDashboard() {
        SmartDashboard.putNumber("Gyro Accel X", gyro.getAccelX());
        SmartDashboard.putNumber("Gyro Accel Y", gyro.getAccelY());
        SmartDashboard.putNumber("Gyro Accel Z", gyro.getAccelZ());
        //the dashboard is smart???? whhhhhaaaarrrrr?????!!!!!!!
        SmartDashboard.putNumber("Gyro Angle X", gyro.getGyroAngleX());
        SmartDashboard.putNumber("Gyro Angle Y", gyro.getGyroAngleY());
        SmartDashboard.putNumber("Gyro Angle Z", gyro.getGyroAngleZ());

        SmartDashboard.putNumber("Gyro Angle: ", gyro.getAngle());

        SmartDashboard.putNumber("Gyro Rate ", gyro.getRate());

        action.dashboardInfo();
    }

    public void initSolenoid(int fc1, int rc1, int fc2, int rc2){
        Stage1Helper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, fc1, rc1); //forward channel, reverse channel
        Stage2Gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, fc2, rc2);
        Stage1Helper.set(kReverse); // real init hours
        breakpiston = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        breakpiston.set(true); // true on jah - DJ
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

    public void inputs() {
        double leftJoystickY = xboxController.getLeftY();
        double leftJoystickX = xboxController.getLeftX();
        double rightJoystickX = xboxController.getRightX();
        double c = 0; // shrimple and inefficient
        double d = 0;
        var STRAFE = 0.0;
        var FORWARD = 0.0;
        var ROTATE = 0.0;
        var multiplier = (-(xboxController.getLeftTriggerAxis() * .3) + (xboxController.getRightTriggerAxis() * .4)
                + .5); // (-RightStick.getThrottle() * 0.5) + 0.5;
        STRAFE = bufferJoystickInput(-leftJoystickX*multiplier, 0.2);
        FORWARD = bufferJoystickInput(-leftJoystickY*multiplier, 0.2); // Joystick direction is opposite
        ROTATE = bufferJoystickInput(rightJoystickX*multiplier, 0.2); // Buffer the input
        drive.driveCartesian(FORWARD, -STRAFE, ROTATE); //THIS IS THAT GUY!!!
        //LiftAxisController.set2(((xboxController2.getLeftTriggerAxis() - xboxController2.getRightTriggerAxis()) * .2));

        //gripperCarriageController.set(ControlMode.PercentOutput,0);
        // theoretically this would let it fight gravity while responding to my
        // controls. Not tuned yet.
        // EDIT: 3 weeks later, its tuned.
        action.analog();

        if (xboxController2.getBButton()) {
            double timeinit = action.getTime();
            action.action(timeinit);
        }

          if (xboxController2.getAButtonPressed()) {
            Stage2Gripper.toggle(); //toggle gripper
        } if (xboxController2.getYButtonPressed()) {
            Stage1Helper.toggle();  //toggle pneumatics
        } if (xboxController2.getXButton()) {
            action.stop();
        }
        if (xboxController.getBButtonPressed()) {
            breakpiston.toggle(); //break yuh
        }

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

        //action(); // showtime!!
    }

    public void autonMode(boolean true_or_false){
        drive.driveCartesian(0, 0, 0);

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

    

    public void gyrocorrect(double setpoint) {
        //please correct thineself sir!
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
    // MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
    //     m_kinematics,
    //     aGyro.getRotation2d(),
    //     new MecanumDriveWheelPositions(
    //     frontLeft.getSelectedSensorPosition(0), frontRight.getSelectedSensorPosition(0),
    //     backLeft.getSelectedSensorPosition(0), backRight.getSelectedSensorPosition(0)
    //     ),
    //     new Pose2d(5.0, 13.5, new Rotation2d())
    // );

    public static void main(String[] args) {
        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(new KeyEventDispatcher() {

            @Override
            public boolean dispatchKeyEvent(KeyEvent ke) {
                synchronized (Robot.class) {
                    switch (ke.getID()) {
                    case KeyEvent.KEY_PRESSED:
                        if (ke.getKeyCode() == KeyEvent.VK_W) {
                            wPressed = true;
                        }
                        break;

                    case KeyEvent.KEY_RELEASED:
                        if (ke.getKeyCode() == KeyEvent.VK_W) {
                            wPressed = false;
                        }
                        break;
                    }
                    return false;
                }
            }
        });
    }
    
}
