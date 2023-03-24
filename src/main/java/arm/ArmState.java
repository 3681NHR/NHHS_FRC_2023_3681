package arm;

public enum ArmState {
    Idle,
    Home,
    Low,
    Medium,
    High,
    SweepStart,
    SweepMiddleA,
    SweepMiddleB,
    SweepMiddleC,
    SweepFinish,
    Recalibrate,
    Extension,
    Dropping;

    public int getId() {
        return this.ordinal();
    }
}