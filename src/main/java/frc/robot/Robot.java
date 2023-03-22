// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController; 
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
//import edu.wpi.first.wpilibj.DigitalInput; //damn!!! digital input, cant remember where I needed that
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.drive;
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
import com.pathplanner.lib.PathPoint;

import motor.SparkWrapper;
import motor.VictorWrapper;
import motor.motorinterface;

import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;
//liveshare link*
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

    private static final int ROTATING_ARM_CONTROLLER_CAN_ID = 6; // still doesnt work \o/ (it works) 3 weeks later
    private static final int ROTATING_ARM_ENCODER_PIN_A = 0; // TODO: Set this properly once it's plugged in
    private static final int ROTATING_ARM_ENCODER_PIN_B = 1; // currently set up to be the gripper carriage encoder

    private static final int GRIPPER_CARRIAGE_CONTROLLER_CAN_ID = 9;
    private static final int GRIPPER_CARRIAGE_ENCODER_PIN_A = 2; //tbd
    private static final int GRIPPER_CARRIAGE_ENCODER_PIN_B = 3; //tbd
    /* Little explanation:
     * SparkWrapper is a class dedicated to specifically the Rev Robotics Sparkmax motor controller
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
    motorinterface frontLeft = new SparkWrapper(FRONT_LEFT_WHEEL_CAN_ID, "frontLeft");
    motorinterface backLeft = new SparkWrapper(BACK_LEFT_WHEEL_CAN_ID, "Back Left");
    motorinterface frontRight = new SparkWrapper(FRONT_RIGHT_WHEEL_CAN_ID, "Front Right");
    motorinterface backRight = new SparkWrapper(BACK_RIGHT_WHEEL_CAN_ID, "Back Right");
    CANSparkMax gripperCarriageController = new CANSparkMax(GRIPPER_CARRIAGE_CONTROLLER_CAN_ID,
            MotorType.kBrushed);
    motorinterface LiftAxisController = new SparkWrapper(ROTATING_ARM_CONTROLLER_CAN_ID, "Rotating Arm");
    motorinterface spinner = new VictorWrapper(16, "spinna real");
    motorinterface spinner2 = new VictorWrapper(17, "real shit"); //I understand everything now 3/21/23

    drive drive = new drive(frontLeft, backLeft, frontRight, backRight); //vrooo
    private Encoder rotatingArmEncoder = new Encoder(ROTATING_ARM_ENCODER_PIN_A,
            ROTATING_ARM_ENCODER_PIN_B,
            false,
            CounterBase.EncodingType.k4X);
    private Encoder gripperCarriageEncoder = new Encoder(GRIPPER_CARRIAGE_ENCODER_PIN_A, 
            GRIPPER_CARRIAGE_ENCODER_PIN_B,
            false,
            CounterBase.EncodingType.k4X);
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
    double actionprocess = 0;
    double heading;
    double gyro_error = 0;
    long t = System.currentTimeMillis();
    long end;
    double fin = -45;

    private final Lock dataLock = new ReentrantLock();
    // TODO: Update gain vals in the feed forward
    private final SimpleMotorFeedforward rotatingArmFeedForward = new SimpleMotorFeedforward(0.38123, 0.07469);
    private final PIDController rotatingArmPIDController = new PIDController(1.0565, 0, 0.23897);

    private static volatile boolean wPressed = false;
    public static boolean isWPressed() {
        synchronized (Robot.class) {
            return wPressed;
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
        initSolenoid(7, 5, 4, 6); //channels for solenoids
        
        initializeRotatingArmEncoder();

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
        //System.out.println(exampleState.velocityMetersPerSecond);

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
        SmartDashboard.putNumber("Encoder Distance", rotatingArmEncoder.getDistance());
        SmartDashboard.putNumber("Encoder Rate", rotatingArmEncoder.getRate());
        SmartDashboard.putNumber("Encoder Raw", rotatingArmEncoder.getRaw());
        SmartDashboard.putNumber("Carriage Encoder Distance", gripperCarriageEncoder.getDistance());
    }
    
    private void initializeRotatingArmEncoder() {
        double rotation = 360.0/2048.0; //the value to get accurate movement in degrees
        rotatingArmEncoder.setDistancePerPulse(rotation);
        rotatingArmEncoder.setSamplesToAverage(5);
        rotatingArmEncoder.setMinRate(0.05);
        rotatingArmEncoder.reset(); // GRAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH
    }

    private void rotateArmToPosition(double SET) {
        // Read 
        // Turn rotatingArm motor controller on yuh
        Stage1Helper.set(kOff);
        double buffer = .0000001;
        double multipler1 = -.5;
        //LiftAxisController.set(ControlMode.Position, SET);
         if ((rotatingArmEncoder.getDistance() <= SET-buffer || rotatingArmEncoder.getDistance() >= SET+buffer)) {
             double differencer = rotatingArmEncoder.getDistance() - SET;
             // god forbid this is wrong - if differencer is negative then it goes down right????
             // and positive means up??? !
             double speedcalc = ((differencer)/(Math.abs(differencer)));
             // actual distance = .1, distance = .45 then .1 - .45 is negative then you have to go
            System.out.println(differencer);
             setRotatingArmSpeed(speedcalc*multipler1); // TODO: Make this speed a bit smarter
             if (speedcalc == 1){
               //  Stage1Helper.set(kForward);
             } else if(speedcalc == -1) {
               //  Stage1Helper.set(kReverse);
             }
         }
    }

    public void setRotatingArmSpeed(double rotatingArmSpeedMetersPerSecond) {
         final double feedForward = rotatingArmFeedForward.calculate(rotatingArmSpeedMetersPerSecond);
        System.out.println(rotatingArmSpeedMetersPerSecond);
        double output =
             rotatingArmPIDController.calculate(-(rotatingArmEncoder.getRate()), -45);
        //  LiftAxisController.setVoltage(output + feedForward); 
        LiftAxisController.set(ControlMode.PercentOutput, output*.1+feedForward*.1);
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
        LiftAxisController.set(ControlMode.PercentOutput,((xboxController2.getLeftTriggerAxis() - xboxController2.getRightTriggerAxis()) * .2));
        //LiftAxisController.set2(((xboxController2.getLeftTriggerAxis() - xboxController2.getRightTriggerAxis()) * .2));
        if (xboxController2.getLeftBumper()) {
            c = 1; // if I give it some more voltage hrmrmm I can make it hold up but it will start
        } else if (xboxController2.getRightBumper()) {
            d = -1.0;
        } else {
            gripperCarriageController.set(0);
            c = 0;
            d = 0;
        }
        gripperCarriageController.set(.05 + c + d);
        //gripperCarriageController.set(ControlMode.PercentOutput,0);
        // theoretically this would let it fight gravity while responding to my
        // controls. Not tuned yet.
        // EDIT: 3 weeks later, its tuned.
        if (xboxController2.getAButtonPressed()) {
            Stage2Gripper.toggle(); //toggle gripper
        }
        if (xboxController2.getYButtonPressed()) {
            Stage1Helper.toggle();  //toggle pneumatics
        }
        if (xboxController2.getXButton()) {
            
        }
        if (xboxController2.getBButton()) {
            action();
        }
        if (xboxController.getBButtonPressed()) {
            breakpiston.toggle(); //break yuh
        }
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

        //action(); // showtime!!
    }

    public void action(){

        if (actionprocess <= .2) {
        }
        rotateArmToPosition(fin); // rotate arm function, input desired distance
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
        //System.out.println(gyro.getGyroAngleX());
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
