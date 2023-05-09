package team3681.robot.subsystem.State.Events;

import team3681.robot.subsystem.State.Events.interfaces.BehaviorManager;
import edu.wpi.first.wpilibj.XboxController;
import team3681.robot.lib.drivebase.MDrive;
// import team3681.robot.lib.hardware.HID.HIDWrapper;
// import team3681.robot.lib.hardware.HID.HIDWrapperBuilder;
import team3681.robot.lib.hardware.motor.interfaces.UniversalMotor;

// NOTE: The command handler takes commands as static classes in command lib.
// I am importing every single command from commandlib using *
import team3681.robot.subsystem.State.Events.Command.CommandHandler;
import team3681.robot.subsystem.State.Events.Command.CommandLib.*;

/**
 * Manages the controls and sends commands to the CommandHandler.
 * <p>
 * Probably should use it once only.
 */
public class TeleopManager implements BehaviorManager {

    //NOTE: Static is used a lot here because I want, 
    // if for some reason you create more than one instance of this,
    // the variables/objects to be shared across instances.

    private static MDrive drive;
    private static UniversalMotor motorA;
    private static UniversalMotor motorB;

    private static XboxController controllerA;
    private static XboxController controllerB;

    volatile boolean isRunning;

    // NOTE: Initializes new command queue thread.
    private static CommandHandler aHandler;

    public TeleopManager(Builder builder) {
        TeleopManager.controllerA = builder.controllerA;
        TeleopManager.controllerB = builder.controllerB;

        TeleopManager.drive = builder.drive;

        TeleopManager.aHandler = new CommandHandler();
    }

    public static class Builder {
        private XboxController controllerA;
        private XboxController controllerB;
        private MDrive drive;

        public Builder() {
        }

        public Builder withMDrive(MDrive drive) {
            this.drive = drive;
            return this;
        }

        public Builder withXboxControllerA(XboxController controllerA) {
            this.controllerA = controllerA;
            return this;
        }

        public Builder withXboxControllerB(XboxController controllerB) {
            this.controllerB = controllerB;
            return this;
        }

        public TeleopManager build() {
            return new TeleopManager(this);
        }
    }

    @Override
    public void initialize() {
        isRunning = true;
        aHandler.notify();
    }

    @Override
    public void cease() throws InterruptedException {
        aHandler.wait();
        isRunning = false;
    }

    @Override
    public boolean isRunning() {
        return isRunning;
    }

    // Declare variables here
    private static double armAdjust;
    private static double driveAdjust;

    private static double cBLTA;
    private static double cALTA;
    private static double cBRTA;
    private static double cARTA;

    private static double leftJoystickY;
    private static double leftJoystickX;
    private static double rightJoystickX;

    private static int leftBumper;
    private static int rightBumper;

    private static double strafe;
    private static double forward;
    private static double rotate;

    private static double carriageSpeedL;
    private static double carriageSpeedR;

    private static final double LOWER_MOD = 0.3;
    private static final double MAGIC_NUMBER_I_DONT_CARE = 0.4;
    private static final double DEFAULT_NUM = 0.7;
    private static final double ZERO_POINT_FIVE = 0.5; //im done.
    private static final double INPUT_BUFFER_AMOUNT = 0.2;

    private static final int BOTH_TRUE = 3;
    private static final int LEFT_TRUE = 2;
    private static final int RIGHT_TRUE = 1;

    private static final double CARRIAGE_SPEED = 0.7;
    private static final double STOPPED_SPEED = 0;

    // NOTE: BELOW IS INPUT CONTROLS

    /**
     * Runs all the input controls for controller A.
     */

    private static void updateValues() {
        // NOTE: The long names suck.
        cBLTA = controllerB.getLeftTriggerAxis();
        cALTA = controllerA.getLeftTriggerAxis();
        cBRTA = controllerB.getRightTriggerAxis();
        cARTA = controllerA.getRightTriggerAxis();

        leftJoystickY = controllerA.getLeftY();
        leftJoystickX = controllerA.getLeftX();
        rightJoystickX = controllerA.getRightX();

        strafe = bufferJoystickInput(-leftJoystickX * driveAdjust, INPUT_BUFFER_AMOUNT);
        forward = bufferJoystickInput(-leftJoystickY * driveAdjust, INPUT_BUFFER_AMOUNT);
        rotate = bufferJoystickInput(rightJoystickX * driveAdjust, INPUT_BUFFER_AMOUNT);
    }

    public static void inputControllerA() {
        updateValues();
        armAdjust = (cBLTA * LOWER_MOD) - (cBRTA * ZERO_POINT_FIVE);

        driveAdjust = (cALTA * LOWER_MOD)
                - (cARTA * MAGIC_NUMBER_I_DONT_CARE)
                + DEFAULT_NUM;

        //NOTE: This is what drives the wheels. Remember that.
        driveCartesian(driveAdjust, strafe, forward, rotate);

        // Xbox controller buttons below probably
        
        // Write code here...

        if (controllerA.getAButtonPressed()) {
            //Example command send
            // Adds the command to the back of the queue
            aHandler.addCommand(new StopCommand());
        }

        if(controllerB.getBButtonPressed()) {

            // Adds the command to the front. Priority queue.
            aHandler.insertCommandToFront(new RealCommand2());
        }

        leftBumper = controllerB.getLeftBumper() ? 1 : 0;
        rightBumper = controllerB.getRightBumper() ? 1 : 0;

        switch (leftBumper * 2 + rightBumper) {
            case BOTH_TRUE: //NOTE: While both true, cancel out all movement
                carriageSpeedL = STOPPED_SPEED;
                carriageSpeedR = STOPPED_SPEED;
                break;

            case LEFT_TRUE:
                carriageSpeedL = CARRIAGE_SPEED;
                carriageSpeedR = STOPPED_SPEED;
                break;

            case RIGHT_TRUE:
                carriageSpeedR = CARRIAGE_SPEED;
                carriageSpeedL = STOPPED_SPEED;
                break;
            default:
                carriageSpeedL = STOPPED_SPEED;
                carriageSpeedR = STOPPED_SPEED;
                break;
        }

        // Write code for using carriage stuff and arm stuff here...

    }

    /**
     * Runs all the input controls for controller A.
     */
    public static void inputControllerB() {
        updateValues();

    }

    private static boolean driveCartesian(double driveAdjust, double F, double S, double R) {
        TeleopManager.drive.driveCartesian(MAGIC_NUMBER_I_DONT_CARE, LOWER_MOD, DEFAULT_NUM);
        return true;
    }

    private static double bufferJoystickInput(double inputValue, double bufferAmt) {
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
