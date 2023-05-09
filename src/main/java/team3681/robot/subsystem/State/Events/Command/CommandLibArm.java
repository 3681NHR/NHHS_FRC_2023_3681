package team3681.robot.subsystem.State.Events.Command;

import edu.wpi.first.wpilibj.drive.RobotDriveBase;

import team3681.robot.lib.drivebase.MDrive;
import team3681.robot.lib.drivebase.PrototypeDrive;
import team3681.robot.lib.drivebase.SDrive;
import team3681.robot.lib.hardware.motor.model.SparkWrapper;
/**
 * CommandArmLib is a library of commands to use.
 * <p>
 * Every static class inside defines one command.
 * <p>
 * I know I am sacrificing simplicity, but this is very modular.
 * <p>
 * If you want to make more, look at the other commands and copy and paste. So shrimple.
 * <p>
 * Currently this uses information for the 2023 robot. Its going to be a pain in the arse
 * @see CommandHandler
 * @see Stuff
 */

 //NOTE: DO NOT USE CODE UNSAFE FOR THREADS IN HERE !!!

public class CommandLibArm {
    private static RobotDriveBase drive;
    private static SparkWrapper MotorA;
    private static SparkWrapper MotorB;
    //NOTE: You probably should be using spark max's
    // if not, then you can be a pain in the arse and sort stuff out themselves

    /**
     * It will sort what drive base yall usin, dw.
     * <p>
     * real!
     * @param drive
     */
    public CommandLibArm(RobotDriveBase drive, SparkWrapper MotorA, SparkWrapper MotorB) {
        if (drive instanceof MDrive) {
            CommandLibArm.drive = (MDrive) drive;
        } else if (drive instanceof PrototypeDrive) {
            CommandLibArm.drive = (PrototypeDrive) drive;
        } else if (drive instanceof SDrive) {
            CommandLibArm.drive = (SDrive) drive;
        } else {
            throw new RuntimeException("I didn't build in support for anything else. Too bad.");
        }

        CommandLibArm.MotorA = MotorA;
        CommandLibArm.MotorB = MotorB;
    }

    public static class StopCommand implements CommandHandler.CommandPointer {
        @Override
        public boolean execute() {
            // Code to stop the robot
            System.out.println("Stopping the robot");
            return false;
        }
    }

    public static class RealCommand2 implements CommandHandler.CommandPointer {
        @Override
        public boolean execute() {
            // Code to stop the robot
            System.out.println("Stopping the robot");
            return false;
        }
    }

}