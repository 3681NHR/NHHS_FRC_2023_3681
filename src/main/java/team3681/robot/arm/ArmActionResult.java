package team3681.robot.arm;

public class ArmActionResult {
    private ArmState state = ArmState.Idle;
    private long delay = 0;
    private boolean changed = false;
    
    public static ArmActionResult noChange() {
        return new ArmActionResult(false);
    }
    
    public static ArmActionResult changeTo(ArmState state) {
        return new ArmActionResult(0, state);
    }
    
    public static ArmActionResult changeToAfter(ArmState state, long delay) {
        return new ArmActionResult(delay, state);
    }
    
    private ArmActionResult(long delay, ArmState state) {
        this.state = state;
        this.delay = delay;
        this.changed = true;
    }
    
    private ArmActionResult(boolean change) {
        this.changed = change;
    }

    public ArmState getState() {
        return state;
    }

    public long getDelay() {
        return delay;
    }

    public boolean isChanged() {
        return changed;
    }
}
