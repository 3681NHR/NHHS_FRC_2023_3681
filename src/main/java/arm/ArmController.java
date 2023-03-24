package arm;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class ArmController {
    private interface ArmAction {
        public default ArmActionResult run(ArmWrapper MainArm) {
            return ArmActionResult.noChange();
        };
    }

    private ArmWrapper mainArm;

    private ArmState currentStateId;

    private ArmState queuedState = null;
    private long queuedTime = 0;

    private static Map<ArmState, ArmAction> states = new HashMap<>();

    /**
     * 
     * @param mainArm      ArmWrapper object
     * @param defaultState null
     */
    public ArmController(ArmWrapper mainArm, ArmState defaultState) {
        requireNonNullParam(mainArm, "MainArm", "ArmController");

        this.mainArm = mainArm;

        if (defaultState == null) {
            this.currentStateId = ArmState.Idle;
        } else {
            this.currentStateId = defaultState;
        }
    }

    public void runPeriodic() {
        ArmAction currentAction = getCurrentAction();
        if (currentAction == null) {
            throw new RuntimeException("Attempted to run action with no corresponding action id");
        }

        ArmActionResult result = currentAction.run(this.mainArm);
        if (this.queuedState == null) {
            if (result != null) {
                if (result.isChanged()) {
                    long delay = result.getDelay();
                    if (delay != 0) {
                        this.queuedState = result.getState();
                        this.queuedTime = System.currentTimeMillis() + delay;
                    } else {
                        this.currentStateId = result.getState();
                    }
                }
            }
        } else {
            if (System.currentTimeMillis() >= this.queuedTime) {
                this.currentStateId = this.queuedState;
                this.queuedState = null;
                this.queuedTime = 0;
            }
        }
    }

    public ArmState getState() {
        return currentStateId;
    }

    public void setState(ArmState state) {
        this.currentStateId = state;
    }

    public void putDashboard() {
        SmartDashboard.putString("State", this.getState().toString());
    }

    private ArmAction getCurrentAction() {
        return ArmController.states.get(this.currentStateId);
    }

    static {
        ArmController.states.put(ArmState.Idle, new ArmAction() {
            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.analogArm(0);

                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.Home, new ArmAction() {
            private final double HOME_ANGLE = 0.0;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                if (MainArm.PIDControlArm(HOME_ANGLE)) {
                    return ArmActionResult.changeTo(ArmState.Idle);
                }
                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.Low, new ArmAction() {
            private final double LOW_ANGLE = -20;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                if (MainArm.PIDControlArm(LOW_ANGLE)) {
                    return ArmActionResult.changeTo(ArmState.Idle);
                }
                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.Medium, new ArmAction() {
            private final double MEDIUM_POINT = -50;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                if (MainArm.PIDControlArm(MEDIUM_POINT)) {
                    return ArmActionResult.changeTo(ArmState.Idle);
                }
                
                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.Recalibrate, new ArmAction() {

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                double differencer = MainArm.getArmAngle();

                MainArm.analogArm(0.1);
                MainArm.analogCarriage(0.1);

                if (Math.abs(differencer) == 0.0) {
                    return ArmActionResult.changeTo(ArmState.Idle);
                }
                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.SweepStart, new ArmAction() {
            private double ARM_ANGLE = -37;
            private double CARRIAGE_ANGLE = -5.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                // TODO: This needs to be tested, not sure if the pid will fail to keep both close enough to their angles
                if (MainArm.PIDControlArm(ARM_ANGLE) && MainArm.PIDControlCarriage(CARRIAGE_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.SweepMiddleA, 250);
                }

                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.SweepMiddleA, new ArmAction() {
            private double ARM_ANGLE = -28;
            private double CARRIAGE_ANGLE = -22;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.spinIn();
                
                // TODO: This needs to be tested, not sure if the pid will fail to keep both close enough to their angles
                if (MainArm.PIDControlArm(ARM_ANGLE) && MainArm.PIDControlCarriage(CARRIAGE_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.SweepMiddleB, 1500);
                }

                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.SweepMiddleB, new ArmAction() {
            private double ARM_ANGLE = -24;
            private double CARRIAGE_ANGLE = -21.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.spinIn();

                // TODO: This needs to be tested, not sure if the pid will fail to keep both close enough to their angles
                if (MainArm.PIDControlArm(ARM_ANGLE) && MainArm.PIDControlCarriage(CARRIAGE_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.SweepMiddleC, 1500);
                }
                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.SweepMiddleC, new ArmAction() {
            private double ARM_ANGLE = -23;
            private double CARRIAGE_ANGLE = -5.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                // TODO: This needs to be tested, not sure if the pid will fail to keep both close enough to their angles
                if (MainArm.PIDControlArm(ARM_ANGLE) && MainArm.PIDControlCarriage(CARRIAGE_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.SweepFinish, 1500);
                }
                return null;
            }
        });

        ArmController.states.put(ArmState.SweepFinish, new ArmAction() {
            private double ARM_ANGLE = -7;
            private double CARRIAGE_ANGLE = -5.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                // TODO: This needs to be tested, not sure if the pid will fail to keep both close enough to their angles
                if (MainArm.PIDControlArm(ARM_ANGLE) && MainArm.PIDControlCarriage(CARRIAGE_ANGLE)) {
                    return ArmActionResult.changeTo(ArmState.Idle);
                }
                return null;
            }
        });

        // TODO: THIS
        ArmController.states.put(ArmState.Extension, new ArmAction(){
            private double ARM_ANGLE = -7;
            // private double CARRIAGE_ANGLE = -5.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                double differencer = MainArm.getCarriageAngle() - ARM_ANGLE;

                if (Math.abs(differencer) <= 3.0) {
                    return ArmActionResult.changeToAfter(ArmState.Dropping, 2000);
                }
                return ArmActionResult.noChange();
            }
        });

        // TODO: THIS
        ArmController.states.put(ArmState.Dropping, new ArmAction(){
            private double ARM_ANGLE = -7;
            // private double CARRIAGE_ANGLE = -5.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                double differencer = MainArm.getArmAngle() - ARM_ANGLE;

                if (Math.abs(differencer) <= 3.0) {
                    return ArmActionResult.changeToAfter(ArmState.Idle, 2000);
                }
                return ArmActionResult.noChange();
            }
        });

    }
}