package team3681.robot.subsystem.State.Events.Command;

import team3681.robot.lib.tools.AtomicDouble;

/**
 * Shared data and stuff
 */
public class CommandSharedData {
    //NOTE: Put your variable stuff here.
    AtomicDouble foo = new AtomicDouble(0);

    public CommandSharedData() {

    }

    // Define a method to update the foo variable
    public void update() {
        double newVal = 0;
        foo.set(newVal);
    }

    // Define a method to read the foo variable
    public double read() {
        return foo.get();
    }

    // Create a separate thread to update the foo variable
    Thread writerThread = new Thread(new Runnable() {
        @Override
        public void run() {
            while (true) {
                update();
            }
        }
    });
    
    public void startWriter() {
        writerThread.start();
    }
    
}
