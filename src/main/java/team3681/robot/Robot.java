// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3681.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * 3681 Robot - all functional code is in RobotContainer
 * 
 * @see RobotContainer 
 * @see TimedRobot
 */

public class Robot extends TimedRobot {

    RobotContainer rContainer = new RobotContainer();

    @Override
    public void disabledInit() {
        rContainer.onDisable();
    }

    @Override
    public void robotInit() {  
        rContainer.onStart();
        System.out.println("Robot Initiated");
    }

    @Override
    public void robotPeriodic() {
        rContainer.putDashboard();
    }

    @Override
    public void teleopInit() {
        rContainer.onStart();
    }

    @Override
    public void teleopPeriodic() {
        rContainer.teleop();

    }

    @Override
    public void autonomousInit() {
        rContainer.onStart();
    }

    @Override
    public void autonomousPeriodic() {
        rContainer.autonomousMode();
    }
}