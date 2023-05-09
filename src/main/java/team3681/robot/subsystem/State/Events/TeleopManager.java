package team3681.robot.subsystem.State.Events;

import team3681.robot.subsystem.State.Events.Command.CommandHandler;
import team3681.robot.subsystem.State.Events.Command.CommandLib;
import team3681.robot.subsystem.State.Events.Command.CommandHandler.CommandPointer;
import team3681.robot.subsystem.State.Events.interfaces.BehaviorManager;
import edu.wpi.first.wpilibj.XboxController;
import team3681.robot.lib.drivebase.MDrive;
import team3681.robot.lib.hardware.HID.HIDWrapper;
import team3681.robot.lib.hardware.HID.HIDWrapperBuilder;
import team3681.robot.lib.hardware.motor.interfaces.UniversalMotor;
import team3681.robot.lib.hardware.motor.model.SparkWrapper;

public class TeleopManager implements BehaviorManager {
    private MDrive drive;
    private UniversalMotor motorA;
    private UniversalMotor motorB;

    private XboxController controllerA;
    private XboxController controllerB;

    private CommandLib commandLib;
    
    volatile boolean isRunning;

    // NOTE: Initializes new command queue thread.
    private CommandHandler aHandler;

    public TeleopManager(Builder builder) {
        this.aHandler = new CommandHandler();
    }

    public static class Builder {
        private String motorName = "";
        private XboxController controllerA;
        private XboxController controllerB;

        public Builder() {
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

}
