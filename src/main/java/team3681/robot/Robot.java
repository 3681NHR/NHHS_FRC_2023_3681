// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3681.robot;

import team3681.robot.lib.drivebase.MDrive;
import team3681.robot.lib.hardware.motor.interfaces.UniversalMotor;
import team3681.robot.lib.hardware.motor.model.SparkWrapper;
import team3681.robot.lib.hardware.motor.model.VictorWrapper;
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
import edu.wpi.first.wpilibj.TimedRobot;
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




/**
 * 3681 Robot - all functional code is in RobotContainer
 * 
 * @see RobotContainer 
 * @see TimedRobot
 */

public class Robot extends TimedRobot {

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

    // NOTE: Regular constants
    private static final double INPUT_BUFFER_AMOUNT = 0.2;

    // NOTE: I/O
    private static final XboxController controllerA = new XboxController(CONTROLLER_USB_PORT_A);
    private static final XboxController controllerB = new XboxController(CONTROLLER_USB_PORT_B);
    private static final GenericHID buttonPanel = new GenericHID(BUTTON_PANEL_USB_PORT);
    //hidWrapper HID = new hidWrapper(controllerA, controllerB, buttonPanel);

    // NOTE: Motors
    private static final UniversalMotor frontLeft = new SparkWrapper(FRONT_LEFT_WHEEL_CAN_ID, "Front Left", true);
    private static final UniversalMotor backLeft = new SparkWrapper(BACK_LEFT_WHEEL_CAN_ID, "Back Left", true);
    private static final UniversalMotor frontRight = new SparkWrapper(FRONT_RIGHT_WHEEL_CAN_ID, "Front Right", true);
    private static final UniversalMotor backRight = new SparkWrapper(BACK_RIGHT_WHEEL_CAN_ID, "Back Right", true);
    
    private static final UniversalMotor spinnerA = new VictorWrapper(SPINNER_A, "Spinner A");
    private static final UniversalMotor spinnerB = new VictorWrapper(SPINNER_B, "Spinner B");

    private static final UniversalMotor armMotor = new SparkWrapper(ARM_CONTROLLER_CAN_ID, "Rotating Arm", true);
    private static final UniversalMotor carriageMotor = new SparkWrapper(CARRIAGE_CONTROLLER_CAN_ID, "Carriage", false);

    // NOTE: Encoders
    private Encoder armEncoder = new Encoder(ARM_ENCODER_PIN_A, ARM_ENCODER_PIN_B, false, CounterBase.EncodingType.k4X);
    private Encoder carriageEncoder = new Encoder(CARRIAGE_ENCODER_PIN_A, CARRIAGE_ENCODER_PIN_B, false, CounterBase.EncodingType.k4X);
    
    // NOTE: Solenoids
    DoubleSolenoid armPistonSolenoid;
    DoubleSolenoid handPistonSolenoid;
    Solenoid brakePistonSolenoid;

    //NOTE: Misc Digital Input
    DigitalInput LSA = new DigitalInput(LIMIT_SWITCH_PORT_A);
    DigitalInput LSB = new DigitalInput(LIMIT_SWITCH_PORT_B);
    
    // NOTE: Drives / Actors
    ArmWrapper MainArm = new ArmWrapper(armEncoder, carriageEncoder, armMotor, carriageMotor, spinnerA, spinnerB);
    MDrive drive = new MDrive(frontLeft, backLeft, frontRight, backRight);
    ArmController armController = new ArmController(MainArm, ArmState.RecalibrateWait);
    ADIS16448_IMU gyro = new ADIS16448_IMU();

    // NOTE: Raspberry PI interface related variables
    // TODO: Read values from the Raspberry Pi vision component
    NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    NetworkTable visionNetworkTable = networkTables.getTable("Vision");

    // NOTE: Variables for auto
    double x = 0.0; // TODO: Figure this from the kp * error
    double y = 0.0; // TODO: Figure this from kp * error
    double z = 0.0; // TODO: Figure this form kp * error
    long Tinit = System.currentTimeMillis()/1000;


    // NOTE: The heading of the robot when starting the motion
    double heading = 0.0;
    double gyroError = 0.0;
    double fin = -70.0;

    Timer rollerTimer = new Timer("Roller Timer");

    @Override
    public void disabledInit() {
        armPistonSolenoid.set(kReverse);
        armController.setState(ArmState.RecalibrateWait);
        MainArm.generalCalibration();

    }

    @Override
    public void robotInit() {
        // NOTE: Reset things
            
        armController.setState(ArmState.RecalibrateWait);

        MainArm.calibrateArm();
        MainArm.calibrateCarriage();
        MainArm.generalCalibration();

        gyro.calibrate();
        gyro.reset();

        initSolenoid(7, 5, 4, 6); // NOTE: channels for solenoids
        
        System.out.println("Robot Initiated");
    }

    @Override
    public void robotPeriodic() {

        putDashboard();
        armController.runPeriodic(); //NOTE: WHY DO WE NEED TWO IT DOESNT MAKE SENSE
        if (!LSA.get()) {MainArm.calibrateArm();}
        if (!LSB.get()) {MainArm.calibrateCarriage();}
        
    }

    @Override
    public void teleopInit() {
        gyro.calibrate();
        gyro.reset();
        armController.setState(ArmState.RecalibrateWait);
        MainArm.generalCalibration();
    }

    @Override
    public void teleopPeriodic() {
        processInputs();
        armController.runPeriodic();
    }

    @Override
    public void autonomousInit() {
        gyro.calibrate();
        gyro.reset();
        armController.setState(ArmState.AutoHigh);
    }

    @Override
    public void autonomousPeriodic() {
        heading = 0;
        gyroError = heading - gyro.getAngle();
        drive.driveCartesian(0, 0, 0);
    }

    // ========(EVERYTHING BELOW IS NOT PART OF TIMED ROBOT FRC STUFF)================================================
    
    private void putDashboard() {
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
        MainArm.putDashboard();
        
        armController.putDashboard();
    }

    public void initSolenoid(int fc1, int rc1, int fc2, int rc2) {
        armPistonSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, fc1, rc1); // NOTE: Forward channel, reverse
                                                                                   // channel
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

    public void processInputs() {
        double carriageSpeedL;
        double carriageSpeedR;

        double leftJoystickY = controllerA.getLeftY();
        double leftJoystickX = controllerA.getLeftX();
        double rightJoystickX = controllerA.getRightX();
        double strafe = 0.0;
        double forward = 0.0;
        double rotate = 0.0;
        double multiplier = (controllerB.getLeftTriggerAxis() * 0.3) - (controllerB.getRightTriggerAxis() * 0.5);
        double adjuster = (-controllerA.getLeftTriggerAxis() * 0.3) + (controllerA.getRightTriggerAxis() * 0.4) + 0.7; // (-RightStick.getThrottle() * 0.5) + 0.5;
        
        // switch (controllerB.getLeftBumper() && controllerB.getRightBumper()) {
        //     case true: //NOTE: While both true, cancel out all movement
        //         carriageSpeedL = 0;
        //         carriageSpeedR = 0;
        //         break;
        
        //     case (!controllerB.getLeftBumper()) && (controllerB.getRightBumper()):
        //         carriageSpeedL = 0.7;
        //         carriageSpeedR = 0;
        //         break;
        
        //     case (controllerB.getLeftBumper()) && (!controllerB.getRightBumper()):
        //         carriageSpeedR = 0.7;
        //         carriageSpeedL = 0;
        //         break;
        
        //     case false: 
        //         carriageSpeedL = 0;
        //         carriageSpeedR = 0;
        //         break;
        
        //     default:
        //         break;
        // }

        // switch (armController.getState()) {
        //     case Analog:
        //         MainArm.analogCarriage(carriageSpeedL - carriageSpeedR);
        //         MainArm.analogArm(multiplier * 2);
        //         break;
        //     default:
        //         break;
        // }


        // NOTE: Joystick direction is opposite
        strafe = bufferJoystickInput(-leftJoystickX * adjuster, INPUT_BUFFER_AMOUNT);
        forward = bufferJoystickInput(-leftJoystickY * adjuster, INPUT_BUFFER_AMOUNT);
        rotate = bufferJoystickInput(rightJoystickX * adjuster, INPUT_BUFFER_AMOUNT);
        SmartDashboard.putNumber("F", forward);
        SmartDashboard.putNumber("S", strafe);
        SmartDashboard.putNumber("R", rotate);

        drive.driveCartesian(forward, -strafe, rotate);

        if (controllerB.getAButtonPressed()) {
            MainArm.spinIn();
            
            StopRollerTask stopRollerTask = new StopRollerTask();
            rollerTimer.schedule(stopRollerTask, 1000); // NOTE: In 2 seconds, run the stop roller task 
        }
        if (controllerB.getBButtonPressed()) {
            MainArm.spinOut();
            
            StopRollerTask stopRollerTask = new StopRollerTask();
            rollerTimer.schedule(stopRollerTask, 1000); // NOTE: In 2 seconds, run the stop roller task 
        }
        if (controllerB.getYButtonPressed()) {
            //armPistonSolenoid.toggle(); // NOTE: Toggle pneumatics
            armController.setState(ArmState.High);
        }
        if (controllerB.getXButtonPressed()) {
            armController.setState(ArmState.SweepStart);
        }
        if (controllerB.getStartButtonPressed()) {
            handPistonSolenoid.toggle();
            armPistonSolenoid.toggle();
        }
        if(controllerB.getBackButton()) {
                armController.setState(ArmState.Analog);
        }
        if (controllerA.getBButtonPressed()) {
            brakePistonSolenoid.toggle();
        } 

        // position ---------");
        int panelJoystickAngle = buttonPanel.getPOV();
        boolean button1_pressed = buttonPanel.getRawButtonPressed(1);
        boolean button2_pressed = buttonPanel.getRawButtonPressed(2);
        boolean button3_pressed = buttonPanel.getRawButtonPressed(3);
        boolean button4_pressed = buttonPanel.getRawButtonPressed(4);
        boolean button5_pressed = buttonPanel.getRawButtonPressed(5);
        boolean button6_pressed = buttonPanel.getRawButtonPressed(6);
        boolean button9_pressed = buttonPanel.getRawButtonPressed(9);
        boolean switch_flicked = buttonPanel.getRawButtonPressed(10);

        // action();
    }

    public void gyroCorrect(double setpoint) {
        // please correct thineself sir!
        if (gyro.getGyroAngleX() > 0.2 + setpoint) {
            z = 0.1;
        } else if (gyro.getGyroAngleX() < -0.1) {
            z = -0.1;
        } else {
            z = 0.0;
        }

        if (gyro.getGyroAngleY() > 0.2 + setpoint) {
            x = -0.1;
        } else if (gyro.getGyroAngleY() < -0.2) {
            x = 0.1;
        } else {
            x = 0.0;
        }
        SmartDashboard.putNumber("new X: ", x);
        SmartDashboard.putNumber("new Y: ", y);
    }

    public void AutoMove() {
        drive.driveCartesian(-0.3,0,0);
    }

    public void AutoMoveCancel() {
        drive.driveCartesian(0,0,0);
    }

    private class StopRollerTask extends TimerTask {

        @Override
        public void run() {
            spinnerA.set(ControlMode.PercentOutput, 0);
            spinnerB.set(ControlMode.PercentOutput, 0);
        }
        
    }
}