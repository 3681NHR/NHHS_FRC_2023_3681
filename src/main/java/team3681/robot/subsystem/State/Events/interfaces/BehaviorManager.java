package team3681.robot.subsystem.State.Events.interfaces;

public interface BehaviorManager {
    // Method to initialize the behavior manager
    public void initialize();

    public void cease() throws InterruptedException;

    public boolean isRunning();
}
