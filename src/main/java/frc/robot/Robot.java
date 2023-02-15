// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;




/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  MotorController frontLeft = new SparkWrapper(3);
  MotorController backLeft = new SparkWrapper(1);
  MotorController frontRight = new SparkWrapper(2);
  MotorController backRight = new SparkWrapper(4);
  Joystick RightStick = new Joystick(0);
  Joystick leftStick = new Joystick(1);
  MecanumDrive drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
  private static final int kJoystickPort = 0;
  private static final int kEncoderPortA = 0;
  private static final int kEncoderPortB = 1;

  private MotorController m_motor;
  private Joystick m_joystick;
  private Encoder m_encoder;

  @Override
  public void robotInit() {
    //m_motor = new CANSparkMax(frontLeft,MotorType.kBrushless);
    RightStick = new Joystick(kJoystickPort);
    m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPRh encoder.
    m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    

    System.out.println("Robot Inited");

  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
    System.out.println("Periodic");
  }
  private final Lock dataLock = new ReentrantLock();
  // offset and distance must both be accessed under a `data_lock` lock!
  private float offset;
  private float distance;

  @Override
  public void teleopPeriodic() {
    dataLock.lock();
    var offset = this.offset;
    var distance = this.distance;
    dataLock.unlock();
    double FORWARD = RightStick.getY();
    double YAW = RightStick.getZ();
    double STRAFE = RightStick.getX();
    //m_motor.set(RightStick.getY());
  
    //motor speed is -1 to 1
    //m_motor.set(1);
    var x = 0.0;
        if (Math.abs(RightStick.getX()) > 0.2) {
            if (RightStick.getX() < 0) {
                x = RightStick.getX() + 0.2;
            } else {
                x = RightStick.getX() - 0.2;
            }
        }
        var y = 0.0;
        if (Math.abs(RightStick.getY()) > 0.2) {
            if (RightStick.getY() < 0) {
                y = RightStick.getY() + 0.2;
            } else {
                y = RightStick.getY() - 0.2;
            }
        }
       var z = 0.0;
        if (Math.abs(leftStick.getX()) > 0.2) {
            if (leftStick.getX() < 0) {
                z = leftStick.getX() + 0.2;
            } else {
                z = leftStick.getX() - 0.2;
            }
        }
        var multiplier = (-RightStick.getThrottle() * 0.5) + 0.5;
        x *= multiplier;
        y *= multiplier;
        z *= multiplier;
        drive.driveCartesian(x, y, z);
        
      /*   // Thumb button
        if (rightStick.getRawButton(2)) {
            shoot.setVoltage(-14.0);
        } else {
            shoot.setVoltage(0.0);
        }*/
        drive.close();
    System.out.println("teleopPeriodic");
    System.out.print(z);
    System.out.print(x);
    System.out.print(y);
  }

}