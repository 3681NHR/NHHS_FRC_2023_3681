package lib.interfaces;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;

import edu.wpi.first.wpilibj.RobotController;

/**
 * The base universal interface for all motors
 * some code borrowed from team 1861.
 * thanks to yall I think I was useful to my team.
 * All the best,
 * 3681
 * 
 * Thanks. Genuinely.
 */
public interface MotorInterface extends IMotorControllerEnhanced {
    String getName();

    ErrorCode configAllSettings(BaseTalonConfiguration allConfigs, int timeoutMs);

    ErrorCode configFactoryDefault(int timeoutMs);

    double getOutputCurrent();

    void stopMotor(ControlMode mode);

    double get();

    /**
     * Common interface for setting the speed of a motor controller.
     *
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    void set2(double speed);

    /**
     * Sets the voltage output of the MotorController. Compensates for the current
     * bus voltage to
     * ensure that the desired voltage is output even if the battery voltage is
     * below 12V - highly
     * useful when the voltage outputs are "meaningful" (e.g. they come from a
     * feedforward
     * calculation).
     *
     * <p>
     * NOTE: This function *must* be called regularly in order for voltage
     * compensation to work
     * properly - unlike the ordinary set function, it is not "set it and forget
     * it."
     *
     * @param outputVolts The voltage to output.
     */
    default void setVoltage(double outputVolts) {
        set2(outputVolts / RobotController.getBatteryVoltage());
    }

}
