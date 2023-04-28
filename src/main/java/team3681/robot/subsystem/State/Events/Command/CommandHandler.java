package team3681.robot.subsystem.State.Events.Command;

import java.util.Deque;
import java.util.ArrayDeque;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.Semaphore;

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
    private final int MAX_QUEUE_SIZE = 1000; // If memory somehow becomes a problem, you are doing something horribly wrong. Either way set this low to save memory or smth. 
    private Semaphore queueSemaphore;

    private Deque<CommandPointer> commandQueue;
    private volatile boolean isRunning;
    private Deque<CommandPointer> commandHistory = new ConcurrentLinkedDeque<>();  //NOTE: For debugging purposes. You cant really undo mechanical actions can you.
    private Thread queueThread;

    /**
     * Creates another thread to run the command queue. I want the capability to run
     * multiple command threads for concurrent commands. Every objct is a new
     * thread. Create more objects for more command subsets
     */
    public CommandHandler() {
        this.commandQueue = new ArrayDeque<>();
        this.isRunning = true;
        this.queueSemaphore = new Semaphore(MAX_QUEUE_SIZE);

        this.commandHistory = new ConcurrentLinkedDeque<>();
        this.queueThread = new Thread(() -> {
            while (isRunning) {
                synchronized (commandQueue) {
                    while (commandQueue.isEmpty()) {
                        try {
                            commandQueue.wait();
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                            System.err.println("Thread interrupted while waiting for command queue");
                        }
                    }
                    CommandPointer command = commandQueue.poll();
                    command.execute();
                    commandHistory.push(command);
                    queueSemaphore.release();
                }
            }
        });
        this.queueThread.start();
    }

    public void addCommand(CommandPointer command) {
        synchronized (commandQueue) {
            commandQueue.offer(command);
            commandQueue.notify(); 
        }
    }

    public void clearCommandQueue() {
        synchronized (commandQueue) {
            commandQueue.clear();
            queueSemaphore.release(MAX_QUEUE_SIZE);

        }
    }
    
    public void insertCommandToFront(CommandPointer command) {
        synchronized (commandQueue) {
            commandQueue.addFirst(command);
            commandQueue.notify(); 
        }
    }

    @FunctionalInterface
    public static interface CommandPointer {
        public void execute();
    }
}
