package team3681.robot.subsystem.State.arm;

public enum ArmState { //NOTE: THIS IS INEFFICIENT AND DOESNT ALLOW FOR MODULARITY
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
    Reel,
    AutoMove,
    AutoMoveCancel,
    AutoHigh,
    AutoExtensionHigh,
    AutoDropping,
    AutoReel,
    AutoRecalibrateWait,
    AutoRecalibrateStageA,
    AutoRecalibrateStageB;

    public int getId() {
        return this.ordinal();
    }
}