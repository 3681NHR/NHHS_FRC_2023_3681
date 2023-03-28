package arm;

public enum ArmState {
    Idle,
    MatchIdle,
    Home,
    Analog,
    Low,
    Medium,
    High,
    SweepStart,
    SweepMiddleA,
    SweepMiddleB,
    SweepMiddleC,
    Finish,
    RecalibrateWait,
    RecalibrateStageA,
    RecalibrateStageB,
    AutoRecalibrateWait,
    AutoRecalibrateStageA,
    AutoRecalibrateStageB,
    ExtensionLow,
    ExtensionMedium,
    ExtensionHigh,
    Dropping,
    Reel,
    AutoMove,
    AutoMoveCancel,
    AutoHigh,
    AutoExtensionHigh,
    AutoDropping,
    AutoReel;

    public int getId() {
        return this.ordinal();
    }
}