package team3681.robot.lib.hardware.motor.model;

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
import team3681.robot.lib.hardware.motor.interfaces.UniversalMotor;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * This class is a thin wrapper around the VictorSPX that reduces CAN bus / CPU overhead.
 * Connects with CTRE VictorSPX motor controllers and adapts it for the universal IGreenMotor.
 *
 * @see UniversalMotor
 * @see VictorSPX
 */
public class VictorWrapper extends VictorSPX implements UniversalMotor {
    double m_setpoint = 0;

    protected String name = "";

    protected final AtomicBoolean isClosed = new AtomicBoolean(false);
    
    protected void throwIfClosed() {
        if (isClosed.get()) {
          throw new IllegalStateException("This SPARK MAX object has previously been closed.");
        }
      }

    /**
     * I am too tired to add more builder functionality. I have no idea what the idle setting configs are.
     *
     * @param deviceNumber [0,62]
     */
    public VictorWrapper(Builder builder) {
        super(builder.port);
        name = builder.motorName;
    }

    public static class Builder {
        private final int port;
        private String motorName = "";

        public Builder(int port) {
            this.port = port;
        }

        public Builder withMotorName(String motorName) {
            this.motorName = motorName;
            return this;
        }

        public VictorWrapper build() {
            return new VictorWrapper(this);
        }
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

    public void setVolt(double speed) {
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
