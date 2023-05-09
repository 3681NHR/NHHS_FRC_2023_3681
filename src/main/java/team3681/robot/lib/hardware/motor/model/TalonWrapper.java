package team3681.robot.lib.hardware.motor.model;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import team3681.robot.lib.hardware.motor.interfaces.UniversalMotor;

public class TalonWrapper extends TalonFX implements UniversalMotor {

    protected String name = "";

    //TODO: literally finish everything
    //builder isnt set up either.
    //look at the other motor files for help.
    public TalonWrapper(int deviceNumber, String motorName) {
        super(deviceNumber);
        name = motorName;
    }

    @Override
    public void putDashboard() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'putDashboard'");
    }
    
    @Override
    public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configSelectedFeedbackSensor'");
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitCfg, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configSupplyCurrentLimit'");
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setStatusFramePeriod'");
    }

    @Override
    public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getStatusFramePeriod'");
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(SensorVelocityMeasPeriod period, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configVelocityMeasurementPeriod'");
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(VelocityMeasPeriod period, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configVelocityMeasurementPeriod'");
    }

    @Override
    public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configVelocityMeasurementWindow'");
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configForwardLimitSwitchSource'");
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configReverseLimitSwitchSource'");
    }

    @Override
    public void set(ControlMode Mode, double demand) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }

    @Override
    public void set(ControlMode Mode, double demand0, DemandType demand1Type, double demand1) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }

    @Override
    public void neutralOutput() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'neutralOutput'");
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setNeutralMode'");
    }

    @Override
    public void setSensorPhase(boolean PhaseSensor) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSensorPhase'");
    }

    @Override
    public void setInverted(boolean invert) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
    }

    @Override
    public void setInverted(InvertType invertType) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
    }

    @Override
    public boolean getInverted() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
    }

    @Override
    public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configOpenloopRamp'");
    }

    @Override
    public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configClosedloopRamp'");
    }

    @Override
    public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configPeakOutputForward'");
    }

    @Override
    public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configPeakOutputReverse'");
    }

    @Override
    public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configNominalOutputForward'");
    }

    @Override
    public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configNominalOutputReverse'");
    }

    @Override
    public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configNeutralDeadband'");
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configVoltageCompSaturation'");
    }

    @Override
    public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configVoltageMeasurementFilter'");
    }

    @Override
    public void enableVoltageCompensation(boolean enable) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'enableVoltageCompensation'");
    }

    @Override
    public double getBusVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getBusVoltage'");
    }

    @Override
    public double getMotorOutputPercent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMotorOutputPercent'");
    }

    @Override
    public double getMotorOutputVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMotorOutputVoltage'");
    }

    @Override
    public double getTemperature() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTemperature'");
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configSelectedFeedbackSensor'");
    }

    @Override
    public ErrorCode configSelectedFeedbackCoefficient(double coefficient, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configSelectedFeedbackCoefficient'");
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal,
            int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configRemoteFeedbackFilter'");
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configRemoteFeedbackFilter'");
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(BaseTalon talonRef, int remoteOrdinal, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configRemoteFeedbackFilter'");
    }

    @Override
    public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configSensorTerm'");
    }

    @Override
    public double getSelectedSensorPosition(int pidIdx) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSelectedSensorPosition'");
    }

    @Override
    public double getSelectedSensorVelocity(int pidIdx) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSelectedSensorVelocity'");
    }

    @Override
    public ErrorCode setSelectedSensorPosition(double sensorPos, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSelectedSensorPosition'");
    }

    @Override
    public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setControlFramePeriod'");
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setStatusFramePeriod'");
    }

    @Override
    public int getStatusFramePeriod(StatusFrame frame, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getStatusFramePeriod'");
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int deviceID, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configForwardLimitSwitchSource'");
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int deviceID, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configReverseLimitSwitchSource'");
    }

    @Override
    public void overrideLimitSwitchesEnable(boolean enable) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'overrideLimitSwitchesEnable'");
    }

    @Override
    public ErrorCode configForwardSoftLimitThreshold(double forwardSensorLimit, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configForwardSoftLimitThreshold'");
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(double reverseSensorLimit, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configReverseSoftLimitThreshold'");
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configForwardSoftLimitEnable'");
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configReverseSoftLimitEnable'");
    }

    @Override
    public void overrideSoftLimitsEnable(boolean enable) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'overrideSoftLimitsEnable'");
    }

    @Override
    public ErrorCode config_kP(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'config_kP'");
    }

    @Override
    public ErrorCode config_kI(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'config_kI'");
    }

    @Override
    public ErrorCode config_kD(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'config_kD'");
    }

    @Override
    public ErrorCode config_kF(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'config_kF'");
    }

    @Override
    public ErrorCode config_IntegralZone(int slotIdx, double izone, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'config_IntegralZone'");
    }

    @Override
    public ErrorCode configAllowableClosedloopError(int slotIdx, double allowableCloseLoopError, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configAllowableClosedloopError'");
    }

    @Override
    public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configMaxIntegralAccumulator'");
    }

    @Override
    public ErrorCode configClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configClosedLoopPeakOutput'");
    }

    @Override
    public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configClosedLoopPeriod'");
    }

    @Override
    public ErrorCode configAuxPIDPolarity(boolean invert, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configAuxPIDPolarity'");
    }

    @Override
    public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIntegralAccumulator'");
    }

    @Override
    public double getClosedLoopError(int pidIdx) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getClosedLoopError'");
    }

    @Override
    public double getIntegralAccumulator(int pidIdx) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getIntegralAccumulator'");
    }

    @Override
    public double getErrorDerivative(int pidIdx) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getErrorDerivative'");
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selectProfileSlot'");
    }

    @Override
    public double getClosedLoopTarget(int pidIdx) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getClosedLoopTarget'");
    }

    @Override
    public double getActiveTrajectoryPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getActiveTrajectoryPosition'");
    }

    @Override
    public double getActiveTrajectoryVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getActiveTrajectoryVelocity'");
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(double sensorUnitsPer100ms, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configMotionCruiseVelocity'");
    }

    @Override
    public ErrorCode configMotionAcceleration(double sensorUnitsPer100msPerSec, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configMotionAcceleration'");
    }

    @Override
    public ErrorCode configMotionSCurveStrength(int curveStrength, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configMotionSCurveStrength'");
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryPeriod(int baseTrajDurationMs, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configMotionProfileTrajectoryPeriod'");
    }

    @Override
    public ErrorCode clearMotionProfileTrajectories() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'clearMotionProfileTrajectories'");
    }

    @Override
    public int getMotionProfileTopLevelBufferCount() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMotionProfileTopLevelBufferCount'");
    }

    @Override
    public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'pushMotionProfileTrajectory'");
    }

    @Override
    public boolean isMotionProfileTopLevelBufferFull() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isMotionProfileTopLevelBufferFull'");
    }

    @Override
    public void processMotionProfileBuffer() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'processMotionProfileBuffer'");
    }

    @Override
    public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMotionProfileStatus'");
    }

    @Override
    public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'clearMotionProfileHasUnderrun'");
    }

    @Override
    public ErrorCode changeMotionControlFramePeriod(int periodMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'changeMotionControlFramePeriod'");
    }

    @Override
    public ErrorCode getLastError() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getLastError'");
    }

    @Override
    public ErrorCode getFaults(Faults toFill) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFaults'");
    }

    @Override
    public ErrorCode getStickyFaults(StickyFaults toFill) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getStickyFaults'");
    }

    @Override
    public ErrorCode clearStickyFaults(int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'clearStickyFaults'");
    }

    @Override
    public int getFirmwareVersion() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFirmwareVersion'");
    }

    @Override
    public boolean hasResetOccurred() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'hasResetOccurred'");
    }

    @Override
    public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configSetCustomParam'");
    }

    @Override
    public int configGetCustomParam(int paramIndex, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configGetCustomParam'");
    }

    @Override
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configSetParameter'");
    }

    @Override
    public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configSetParameter'");
    }

    @Override
    public double configGetParameter(ParamEnum paramEnum, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configGetParameter'");
    }

    @Override
    public double configGetParameter(int paramEnum, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configGetParameter'");
    }

    @Override
    public int getBaseID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getBaseID'");
    }

    @Override
    public int getDeviceID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDeviceID'");
    }

    @Override
    public ControlMode getControlMode() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getControlMode'");
    }

    @Override
    public void follow(IMotorController masterToFollow) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'follow'");
    }

    @Override
    public void valueUpdated() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'valueUpdated'");
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getName'");
    }

    @Override
    public ErrorCode configAllSettings(BaseTalonConfiguration allConfigs, int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configAllSettings'");
    }

    @Override
    public ErrorCode configFactoryDefault(int timeoutMs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configFactoryDefault'");
    }

    @Override
    public double getOutputCurrent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getOutputCurrent'");
    }

    @Override
    public void stopMotor(ControlMode mode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopMotor'");
    }

    @Override
    public double get() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'get'");
    }

    @Override
    public void setVolt(double speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVolt'");
    }
    
}
