package team3681.robot.lib.drivebase;

import java.io.IOException;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

import team3681.robot.lib.hardware.motor.interfaces.UniversalMotor;

/**
 * A class for the prototype half-swerve drivetrain built during the 2023.
 * The protype consists of the front two wheels being motor-driven swerve
 * wheels,
 * and the back half consistening of two non-motorized omni wheels.
 * 
 * <pre>
 *[o] - [o]
 *|       |
 *|       |
 *{} - - {}
 * </pre>
 * 
 * Developed soley to barely run the design.
 * 
 * @notice Not to be used for actual robots. Please don't.
 */
public class PrototypeDrive extends RobotDriveBase implements Sendable, AutoCloseable {

    private static int instances;

    private final UniversalMotor m_frontLeftMotor;
    private final UniversalMotor m_frontRightMotor;

    private boolean m_reported;

    public PrototypeDrive(
            UniversalMotor frontLeftMotor,

            UniversalMotor frontRightMotor) {
        requireNonNullParam(frontLeftMotor, "frontLeftMotor", "PrototypeDrive");
        requireNonNullParam(frontRightMotor, "frontRightMotor", "PrototypeDrive");

        m_frontLeftMotor = frontLeftMotor;
        m_frontRightMotor = frontRightMotor;
        SendableRegistry.addChild(this, m_frontLeftMotor);
        SendableRegistry.addChild(this, m_frontRightMotor);
        instances++;
        SendableRegistry.addLW(this, "PrototypeDrive", instances);
    }

    @SuppressWarnings("MemberName")
    public static class WheelSpeeds {
        public double frontLeft;
        public double frontRight;

        public WheelSpeeds() {
        }

        public WheelSpeeds(double frontLeft, double frontRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
        }
    }

    public void driveExperimental(double xSpeed, double ySpeed, double zRotation) {
        driveExperimental(xSpeed, ySpeed, zRotation, new Rotation2d());
    }

    public void driveExperimental(double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle) {

        xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);
        ySpeed = MathUtil.applyDeadband(ySpeed, m_deadband);
        
        var speeds = speedCalculator(xSpeed, ySpeed, zRotation, gyroAngle);

        m_frontLeftMotor.setVolt(speeds.frontLeft * m_maxOutput);
        m_frontRightMotor.setVolt(speeds.frontRight * m_maxOutput * -1);

    }

    public static WheelSpeeds speedCalculator(
            double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle) {
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);

        // Compensate for gyro angle.
        var input = new Translation2d(xSpeed, ySpeed).rotateBy(gyroAngle.unaryMinus());

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[MotorType.kFrontLeft.value] = input.getX() + input.getY() + zRotation;
        wheelSpeeds[MotorType.kFrontRight.value] = input.getX() - input.getY() - zRotation;

        normalize(wheelSpeeds);

        return new WheelSpeeds(
                wheelSpeeds[MotorType.kFrontLeft.value],
                wheelSpeeds[MotorType.kFrontRight.value]);

    }

    @Override
    public void stopMotor() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopMotor'");
    }

    @Override
    public String getDescription() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDescription'");
    }

    @Override
    public void close() throws IOException {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initSendable'");
    }

}
