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
    ExtensionLow,
    ExtensionMedium,
    ExtensionHigh,
    Dropping,
    Reel;

    public int getId() {
        return this.ordinal();
    }
}