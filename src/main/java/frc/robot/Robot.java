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
//liveshare link
/**

I blame neil for this existing. - DJ

 * <p>
 * In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent
 * to the Dashboard.
 */

public class Robot extends TimedRobot {
    Thread m_visionThread;
    long t = System.currentTimeMillis();
    long end;
    private static final int FRONT_LEFT_WHEEL_CAN_ID = 3; 
    private static final int BACK_LEFT_WHEEL_CAN_ID = 1; 
    private static final int FRONT_RIGHT_WHEEL_CAN_ID = 2; 
    private static final int BACK_RIGHT_WHEEL_CAN_ID = 4;
    
    private static final int ROTATING_ARM_CONTROLLER_CAN_ID = 6;
    private static final int ROTATING_ARM_ENCODER_PIN_A = 2; // TODO: Set this properly once it's plugged in
    private static final int ROTATING_ARM_ENCODER_PIN_B = 3; // currently set up to be the gripper carriage encoder still doesnt work

    private static final int GRIPPER_CARRIAGE_CONTROLLER_CAN_ID = 9;
    private static final int GRIPPER_CARRIAGE_ENCODER_PIN_A = 1; // 
    private static final int GRIPPER_CARRIAGE_ENCODER_PIN_B = 0; // 
    
    private static final int XBOX_CONTROLLER_USB_PORT = 0; 
    private static final int BUTTON_PANEL_USB_PORT = 4;
    
    ADIS16448_IMU gyro = new ADIS16448_IMU();
    private double[] gyroAccelData = {0.0, 0.0, 0.0};
    private double[] gyroAngleData = {0.0, 0.0, 0.0};
    MotorController frontLeft = new SparkWrapper(FRONT_LEFT_WHEEL_CAN_ID);
    MotorController backLeft = new SparkWrapper(BACK_LEFT_WHEEL_CAN_ID);
    MotorController frontRight = new SparkWrapper(FRONT_RIGHT_WHEEL_CAN_ID);
    MotorController backRight = new SparkWrapper(BACK_RIGHT_WHEEL_CAN_ID);
    
    XboxController xboxController = new XboxController(XBOX_CONTROLLER_USB_PORT);
    XboxController xboxController2 = new XboxController(1);
    GenericHID buttonPanel = new GenericHID(BUTTON_PANEL_USB_PORT);

    MecanumDrive drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    
    private Encoder rotatingArmEncoder = new Encoder(ROTATING_ARM_ENCODER_PIN_A, 
    ROTATING_ARM_ENCODER_PIN_B, 
    false, 
    Encoder.EncodingType.k2X);
    //MotorController rotatingArmController = new SparkWrapper(ROTATING_ARM_CONTROLLER_CAN_ID); //cant have two motor controller things on. Switch this back to RO...etc when the encoder magically works
    CANSparkMax LiftAxisController;

    private Encoder gripperCarriageEncoder; 
    private CANSparkMax gripperCarriageController = new CANSparkMax(GRIPPER_CARRIAGE_CONTROLLER_CAN_ID, MotorType.kBrushed);

    private RelativeEncoder LiftAxisEncoder;
    private SparkMaxPIDController LiftAxisPID;
    
    int a = 0;
    DoubleSolenoid Stage1Helper;
    DoubleSolenoid Stage2Gripper;
    Solenoid breakpiston;
    double pos1;
    double sim;
    double rotations;
    double hardcodeddistance;
    NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    // We'll use this to read values from the Raspberry Pi vision component
    NetworkTable visionNetworkTable = networkTables.getTable("vision");
    // Use gyro declaration from above here

    // The gain for a simple P loop
    double kP = 0.1; 
    double kI = 1e-4;
    double kD = 1; 
    double kIz = 0; 
    double kFF = 0; 
    double kMaxOutput = 1; 
    double kMinOutput = -1;

    // The heading of the robot when starting the motion
    double heading;
    double gyro_error = 0;

    // Initialize motor controllers and drive

    MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, backLeft);
    MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, backRight);


    // Gains are for example purposes only - must be determined for your own robot!
    // TODO: Update gain vals in the feed forward 
    private final SimpleMotorFeedforward rotatingArmFeedForward = new SimpleMotorFeedforward(1, 3);
    private final PIDController rotatingArmPIDController = new PIDController(1, 1, 0);

    // private DigitalInput testLimitSwitch = new DigitalInput(0);

    private void initializeRotatingArmEncoder() {
         // TODO: Check if this is the Encoding type we want
        // Use SetDistancePerPulse to set the multiplier for GetDistance
        // This is set up assuming a 6 inch wheel with a 360 CPRh encoder.

        rotatingArmEncoder.setDistancePerPulse((Math.PI * 6) / 360.0);

        LiftAxisEncoder.setPosition(0);

        rotatingArmEncoder.reset(); //GRAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH
    }

    private void initializeLastDitchEncoderAnalogPID() {
            // PID coefficients
        // for future lookers, we never used this.
       LiftAxisPID = LiftAxisController.getPIDController();

        // set PID coefficients
        LiftAxisPID.setP(kP);
        LiftAxisPID.setI(kI);
        LiftAxisPID.setD(kD);
        LiftAxisPID.setIZone(kIz);
        LiftAxisPID.setFF(kFF);
        LiftAxisPID.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
        
    }

    @Override
    public void autonomousInit() {
        end = System.currentTimeMillis()+2000;
        hardcodeddistance = 0;

    }

    //
    @Override
    public void autonomousPeriodic() {

        // TODO here:
        // Calculate desired heading from the vision module
        // heading = 45;

        gyro_error = heading - gyro.getAngle();
        gyro.getAccelX();

        double x = 0; // TODO: Figure this from the kp * error
        double y = 0; // TODO: Figure this from kp * error
        double z = 0; // TODO: Figure this form kp * error
    }
    @Override
    public void robotInit() {
        //here
        gyro.calibrate();
        //Shuffleboard.getTab("Fun Stuff").add(gyro);

        //owo - misha
        Stage1Helper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 5);
        Stage2Gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 6);
        Stage1Helper.set(kReverse); //real init hours

        breakpiston = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        breakpiston.set(true); //true on jah - DJ
        pos1 = 0;
        sim = 0;

        //Worst case scenario setup for the NEO brushless encoder.
        LiftAxisController = new CANSparkMax(ROTATING_ARM_CONTROLLER_CAN_ID, MotorType.kBrushless);
        LiftAxisEncoder = LiftAxisController.getEncoder();



        //rotatingArmController.set(0.0);
        
        m_visionThread =
        new Thread(
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
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  //outputStream.notifyError(cvSink.getError());
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
    m_visionThread.setDaemon(true); //haha daemon!
    m_visionThread.start();
        
        // get the library for the ADIS working. (FIXED but im leaving the comment in because I like how awful this code looks)
        // I cannot install it and its causing me issues.

        initializeRotatingArmEncoder();

      initializeLastDitchEncoderAnalogPID();
        
        // Set setpoint to current heading at start of auto
        heading = gyro.getAngle();

        // Invert the right side motors.
        // You may need to change or remove this to match your robot.
        frontRight.setInverted(true);
        backRight.setInverted(true);

        System.out.println("Robot Inited");

        //yuh some demo code I slapped in here
        
}

    /*
     * The RobotPeriodic function is called every control packet no matter the
     * robot mode.
     */
    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Encoder", rotatingArmEncoder.getDistance());
        
        SmartDashboard.putNumber("Gyro2", gyro.getRate());
        
        // System.out.println("Periodic");
    }

    private void putGyroDataOnDashboard() {
        // gyroAccelData[0] = gyro.getAccelX();
        // gyroAccelData[1] = gyro.getAccelY();
        // gyroAccelData[2] = gyro.getAccelZ();
//        SmartDashboard.putNumberArray("Gryo Accel", gyroAccelData);
        SmartDashboard.putNumber("Gyro Accel X", gyro.getAccelX());
        SmartDashboard.putNumber("Gyro Accel Y", gyro.getAccelY());
        SmartDashboard.putNumber("Gyro Accel Z", gyro.getAccelZ());

        // gyroAngleData[0] = gyro.getGyroAngleX();
        // gyroAngleData[1] = gyro.getGyroAngleY();
        // gyroAngleData[2] = gyro.getGyroAngleZ();
        // SmartDashboard.putNumberArray("Gryo Angles", gyroAngleData);

        SmartDashboard.putNumber("Gyro Angle X", gyro.getGyroAngleX());
        SmartDashboard.putNumber("Gyro Angle Y", gyro.getGyroAngleY());
        SmartDashboard.putNumber("Gyro Angle Z", gyro.getGyroAngleZ());

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
    int test = 1;
    @Override
    public void teleopPeriodic() {
        dataLock.lock();
        dataLock.unlock();
        SmartDashboard.putNumber("testnumber", test++);
        double leftJoystickY = xboxController.getLeftY();

        double leftJoystickX = xboxController.getLeftX();

        double rightJoystickX = xboxController.getRightX();
        //System.out.println(rotatingArmEncoder.getDistance()+" arm rotation angle position ---------");
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
        //System.out.println(pos1+" X pressed | rotations: " + rotations+ " | position: "+LiftAxisEncoder.getPosition()+" | carriage enc: "+rotatingArmEncoder.getDistance());
        //System.out.println(gyro.getAccelX()+" x accel | " + gyro.getAccelY() + " y accel | " + gyro.getAccelZ() + " z accel | " + gyro.getYComplementaryAngle() + " y angle | " + System.currentTimeMillis() + " time ");
        putGyroDataOnDashboard();

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

        var multiplier = (-(xboxController.getLeftTriggerAxis()*.4)+(xboxController.getRightTriggerAxis()*.5)+.5);// (-RightStick.getThrottle() * 0.5) + 0.5;
        
        FORWARD *= multiplier;
        STRAFE *= multiplier;
        ROTATE *= multiplier;
        drive.driveCartesian(FORWARD, -STRAFE, ROTATE);

        SmartDashboard.putNumber("testnumber2", test++);
        SmartDashboard.putNumber("Rotating Arm Encoder", rotatingArmEncoder.getDistance());
        
        LiftAxisController.set(((xboxController2.getLeftTriggerAxis()-xboxController2.getRightTriggerAxis())*.2));
        double c = 0; //shrimple and inefficient
        double d = 0;
        // if I give it some more voltage hrmrmm I can make it hold up but it will start slipping
        if(xboxController2.getLeftBumper()) {
            gripperCarriageController.set(1.0);
            //System.out.println("carriage.");
            c=1;
        } 
        else if(xboxController2.getRightBumper()) {
            gripperCarriageController.set(-1.0);
            d=-1.0;
        } else {
            gripperCarriageController.set(0);
            c = 0;
            d = 0;
        }
        if(xboxController.getLeftBumper()) {
            System.out.print("You are an idiot. Stop it.");
        } 
        else if(xboxController.getRightBumper()) {
            System.out.print("You are an idiot. Stop it.");
        }
        
        gripperCarriageController.set(.05+c+d); //TODO: re-enable when the window motor is fixed
        //theoretically this would let it fight gravity while responding to my controls. Not tuned yet.


        if(xboxController2.getAButtonPressed()) {
           a+=1;
        }

        if(a%2==0){
            Stage2Gripper.set(kForward); //toggle switch
        } else {
            Stage2Gripper.set(kReverse); 
        }

        if(xboxController2.getYButtonPressed()) {
            Stage1Helper.toggle();
        }
       

        //Stage1Helper.set(kForward); // <----------
        
       
        if(xboxController.getBButtonPressed()) {
            pos1 = 0;
            breakpiston.toggle();

        }
        if(xboxController2.getXButton()) {
            sim =1.0;
            
            SmartDashboard.putNumber("pos1", pos1);
            
            //LastDitchAnalogControl(pos1);
            //LiftAxisController.set(pos1);

        } else {
            sim = 0.0;
            
            // If X is not pressed, stop the arm from moving
            //LiftAxisEncoder.setPosition(0);
           // rotatingArmController.set(0.0); 
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

       // System.out.println("FORWARD: " + FORWARD + " STRAFE: " + STRAFE + " ROTATE: " + ROTATE);

        SmartDashboard.putNumber("Gyro Rate ", gyro.getRate());
        SmartDashboard.putNumber("Encoder Distance", rotatingArmEncoder.getDistance());
        SmartDashboard.putNumber("Encoder Rate", rotatingArmEncoder.getRate());  
        SmartDashboard.putNumber("Encoder Raw", rotatingArmEncoder.getRaw());  

        //SmartDashboard.putBoolean("limitSwitch", testLimitSwitch.get());
    }

    
    private void LastDitchAnalogControl(double position) { //it holds its own arm fine anyways, honestly wouldnt help us now
        // Read 
        // Turn rotatingArm motor controller on
        
        double buffer = .01;
        double multipler1 = -.1;
        if ((LiftAxisEncoder.getPosition() <= position-buffer || LiftAxisEncoder.getPosition() >= position+buffer) && sim==1.0) {
            double differencer = LiftAxisEncoder.getPosition() - position;
            // please for the love of god work without backlash issues, I am coping hard.
            double speedcalc = ((differencer)/(Math.abs(differencer)));
            // so shrimple its unreal (not) - DJ
            
            AnalogSpeed(speedcalc*multipler1); // TODO: Make this speed a bit smarter
            
            if (speedcalc == 1){
                //Stage1Helper.set(kForward);
            } else if(speedcalc == -1) {
               
            } 

        }
    }

    

    
    public void AnalogSpeed(double rate ){
        final double feedForward = LiftAxisPID.getFF();
        rotations = rate;
       // final double output =
       //     LiftAxisPID.calculate(LiftAxisE.getVelocity(), rate);
        //LiftAxisController.setVoltage(LiftAxisPID.getOutputMax() + feedForward); 
    LiftAxisPID.setReference(rotations, CANSparkMax.ControlType.kPosition);
    

    System.out.print(rotations);
    
    }

    public void AnalogPID(){
        // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { LiftAxisPID.setP(p); kP = p; }
    if((i != kI)) { LiftAxisPID.setI(i); kI = i; }
    if((d != kD)) { LiftAxisPID.setD(d); kD = d; }
    if((iz != kIz)) { LiftAxisPID.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { LiftAxisPID.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      LiftAxisPID.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", LiftAxisEncoder.getPosition());
    SmartDashboard.putNumber("Output", LiftAxisController.getAppliedOutput());

    }

    
    // above is last case scenario (not)
    // everything below is for the encoder that doesnt work. Please be advised, I hate magical errors like this.

    public void setRotatingArmSpeed(double rotatingArmSpeedMetersPerSecond) {
        final double feedForward = rotatingArmFeedForward.calculate(rotatingArmSpeedMetersPerSecond);
        
        final double output =
            rotatingArmPIDController.calculate(rotatingArmEncoder.getRate(), rotatingArmSpeedMetersPerSecond);
        //rotatingArmController.setVoltage(output + feedForward);

      }
    
    private void rotateArmToPosition(double distance) {
        // Read 
        // Turn rotatingArm motor controller on 
       // Stage1Helper.set(kOff);
        double buffer = .1;
        double multipler1 = -0.01;
        if ((rotatingArmEncoder.getDistance() <= distance-buffer || rotatingArmEncoder.getDistance() >= distance+buffer) && sim == 1.0) {
            double differencer = rotatingArmEncoder.getDistance() - distance;
            // god forbid this is wrong - if differencer is negative then it goes down right????
            // and positive means up???
            double speedcalc = ((differencer)/(Math.abs(differencer)));
            // actual distance = .1, distance = .45 then .1 - .45 is negative then you have to go

            setRotatingArmSpeed(speedcalc * multipler1); // TODO: Make this speed a bit smarter
           // Stage1Helper.set(kForward);
            /*if (speedcalc == 1){
                Stage1Helper.set(kForward);
            } else if(speedcalc == 1) {
                Stage1Helper.set(kReverse);
            } */
            System.out.println(speedcalc*multipler1);
        }
        SmartDashboard.putNumber("Encoder Distance", rotatingArmEncoder.getDistance());
        SmartDashboard.putNumber("Encoder Rate", rotatingArmEncoder.getRate());  
    }
    public double integralfunction(double accel){
        //yuh angus hours
        
        double positionoffset;
        double velocity;
        long time = System.currentTimeMillis();

        positionoffset = ((1 / 2) * accel * time * time); //theoretically how physics work
        
        return(positionoffset);
    }
}