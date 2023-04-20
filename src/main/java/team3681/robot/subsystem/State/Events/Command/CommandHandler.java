package team3681.robot.subsystem.State.Events.Command;

import java.util.Queue;
import java.util.LinkedList;

/**
 * This class is created to implement the Command pattern by encapsulating a
 * bunch of commands as objects.
 * <p>
 * Each command is represented as an object of a separate class that implements
 * the Command interface.
 * <p>
 * Commands can be queued up and executed one-by-one by calling the execute()
 * method of each command object.
 * <p>
 * This class may also facilitate the definition of batch commands.
 * <p>
 * (Queuing a set of commands in order)
 */
public class CommandHandler {
    private Queue<CommandPointer> commandQueue;
    private Thread queueThread;

    /**
     * Creates another thread to run the command queue. I want the capability to run
     * multiple command threads for concurrent commands. Every objct is a new
     * thread. Create more objects for more command subsets
     * <p>
     * Example:
     * <p>
     * CommandHandler handler = new CommandHandler();
     * <p>
     * CommandHandler.CommandPointer command = new RealCommand2();
     * <p>
     * handler.addCommand(command);
     * <p>
     * handler.executeCommands(); // Prints "Stopping the robot"
     * 
     */
    public CommandHandler() {
        this.commandQueue = new LinkedList<>();
        this.queueThread = new Thread(() -> {
            while (true) {
                synchronized (commandQueue) {
                    while (commandQueue.isEmpty()) {
                        try {
                            commandQueue.wait();
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    CommandPointer command = commandQueue.poll();
                    command.execute();
                }
            }
        });
        this.queueThread.start();
    }

    public void addCommand(CommandPointer command) {
        synchronized (commandQueue) {
            commandQueue.offer(command);
            commandQueue.notify(); // Notify the waiting thread that there is a new command in the queue
        }
    }

    public static interface CommandPointer {
        public void execute();
    }
}
