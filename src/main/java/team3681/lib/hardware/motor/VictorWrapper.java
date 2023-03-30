package team3681.lib.hardware.motor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.REVLibError;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DriverStation;

import team3681.lib.hardware.interfaces.MotorInterface;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * This class is a thin wrapper around the VictorSPX that reduces CAN bus / CPU overhead.
 * Connects with CTRE VictorSPX motor controllers and adapts it for the universal IGreenMotor.
 *
 * @see MotorInterface
 * @see VictorSPX
 */
public class VictorWrapper extends VictorSPX implements MotorInterface {
    double m_setpoint = 0;

    protected String name = "";

    protected final AtomicBoolean isClosed = new AtomicBoolean(false);

    protected void throwIfClosed() {
        if (isClosed.get()) {
          throw new IllegalStateException("This SPARK MAX object has previously been closed.");
        }
      }

    /**
     * Constructor
     *
     * @param deviceNumber [0,62]
     */
    public VictorWrapper(int deviceNumber, String motorName) {
        super(deviceNumber);
        name = motorName;
    }
    @Override
    public void stopMotor(ControlMode mode){
        throwIfClosed();
        set(ControlMode.Current, 0);
    }

    @Override
    public double get() {
      throwIfClosed();
      return m_setpoint;
      /* This doesnt do anything  */
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(
        SupplyCurrentLimitConfiguration currLimitCfg,
        int timeoutMs
    ) {
        DriverStation.reportWarning(
            "method: configSupplyCurrentLimit not implemented for LazyVictorSPX",
            false
        );
        return null;
    }

    @Override
    public ErrorCode setStatusFramePeriod(
        StatusFrameEnhanced frame,
        int periodMs,
        int timeoutMs
    ) {
        DriverStation.reportWarning(
            "method: setStatusFramePeriod not implemented for LazyVictorSPX",
            false
        );
        return null;
    }

    @Override
    public double getOutputCurrent() {
        DriverStation.reportWarning(
            "method: getOutputCurrent not implemented for LazyVictorSPX",
            false
        );
        return 0;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(
        LimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int timeoutMs
    ) {
        DriverStation.reportWarning(
            "method: configReverseLimitSwitchSource not implemented for LazyVictorSPX",
            false
        );
        return null;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public ErrorCode configAllSettings(BaseTalonConfiguration allConfigs, int timeoutMs) {
        System.out.println(
            "WARNING: configAllSettings not implemented in LazyVictorSPX!"
        );
        return ErrorCode.OK;
    }

    public void set2(double speed) {
        throwIfClosed();
        // Only for 'get' API
        m_setpoint = speed;
        setpointCommand(speed, ControlType.kDutyCycle);
    }

    REVLibError setpointCommand(double value, ControlType controlType) {
        throwIfClosed();
        return setpointCommand(value, ControlType.kDutyCycle);
    }
}
