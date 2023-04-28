package team3681.robot.subsystem.State.Events;

import team3681.robot.subsystem.State.Events.Command.CommandHandler;
import team3681.robot.subsystem.State.Events.Command.CommandLib;
import team3681.robot.subsystem.State.Events.Command.CommandHandler.CommandPointer;
import team3681.robot.subsystem.State.Events.interfaces.BehaviorManager;

import team3681.robot.lib.drivebase.MDrive;
import team3681.robot.lib.hardware.motor.interfaces.UniversalMotor;
import team3681.robot.lib.hardware.motor.model.SparkWrapper;

public class TeleopManager implements BehaviorManager{
    MDrive drive;
    UniversalMotor motorA;
    UniversalMotor motorB;

    CommandLib commandLib;
    
    //NOTE: Initializes new command queue thread.
    CommandHandler TeleopManager = new CommandHandler();
    public TeleopManager() {
        
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initialize'");
    }
    
    
}
