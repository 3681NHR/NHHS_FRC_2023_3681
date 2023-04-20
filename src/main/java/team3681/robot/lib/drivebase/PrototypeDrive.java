package team3681.robot.lib.drivebase;

import java.io.Closeable;
import java.io.IOException;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

/**
 * A class for the prototype half-swerve drivetrain built during the 2023.
 * The protype consists of the front two wheels being motor-driven swerve wheels,
 * and the back half consistening of two non-motorized omni wheels.
 * <pre>
 * [o] - [o]
 *|       |
 *|       |
 *{} - - {}
 * </pre>
 * 
 * Developed soley to barely run the design.
 * 
 * @deprecated Not to be used for actual robots.
 */
public class PrototypeDrive extends RobotDriveBase implements Sendable, AutoCloseable{

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
