package team3681.robot.subsystem.State.Events.interfaces;

public interface BehaviorManager {
    // Method to initialize the behavior manager
    public void initialize();

    // Method to get a behavior by name from the behavior manager
    //public Behavior getBehavior(String name);

    // Method to execute a behavior by name
    public void executeBehavior(String name);

    // Method to stop a behavior by name
    public void stopBehavior(String name);

    public void addBehavior(String name);
}
