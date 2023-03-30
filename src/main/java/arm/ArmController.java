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
        long currentTime = System.currentTimeMillis();
        ArmAction currentAction = getCurrentAction();
        
        if (currentAction == null) {
            throw new RuntimeException("Attempted to run action with no corresponding action id");
        }
        
        ArmActionResult result = currentAction.run(this.mainArm);
        if (this.queuedState != null) {
            if (currentTime >= this.queuedTime) {
                this.currentStateId = this.queuedState;
                this.queuedState = null;
                this.queuedTime = 0;
            }
        } else if (result != null && result.isChanged()) {
            long delay = result.getDelay();
            if (delay != 0) {
                this.queuedState = result.getState();
                this.queuedTime = currentTime + delay;
            } else {
                this.currentStateId = result.getState();
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

    static { // NOTE: TERMINATE ALL MOVEMENT
        ArmController.states.put(ArmState.Idle, new ArmAction() {
            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.terminateArm();
                MainArm.terminateCarriage();
                MainArm.spinStop();
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

        ArmController.states.put(ArmState.Analog, new ArmAction() {

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.Low, new ArmAction() {
            private final double LOW_ANGLE = -25;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                if (MainArm.PIDControlArm(LOW_ANGLE)) {
                    return ArmActionResult.changeTo(ArmState.ExtensionLow);
                }
                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.Medium, new ArmAction() {
            private final double MEDIUM_POINT = -50;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                if (MainArm.PIDControlArm(MEDIUM_POINT)) {
                    return ArmActionResult.changeTo(ArmState.ExtensionMedium);
                }

                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.High, new ArmAction() {
            private final double MEDIUM_POINT = -94;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                if (MainArm.PIDControlArm(MEDIUM_POINT)) {
                    return ArmActionResult.changeTo(ArmState.ExtensionHigh);
                }

                return ArmActionResult.noChange();
            }
        });
        ArmController.states.put(ArmState.RecalibrateWait, new ArmAction() {

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.analogArm(0.3);
                MainArm.analogCarriage(0.5);
                return ArmActionResult.changeToAfter(ArmState.RecalibrateStageA, 250);
            }
        });

        ArmController.states.put(ArmState.RecalibrateStageA, new ArmAction() {

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {

                if (MainArm.getCarriageAngle() == 0.0) {
                    MainArm.analogCarriage(0.0);
                    return ArmActionResult.changeToAfter(ArmState.RecalibrateStageB, 500);
                } else {
                    MainArm.analogCarriage(0.8);
                }
                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.RecalibrateStageB, new ArmAction() {

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {

                if (MainArm.getArmAngle() == 0.0) {
                    MainArm.analogArm(0.0);
                    return ArmActionResult.changeTo(ArmState.MatchIdle);
                } else {
                    MainArm.analogArm(0.3);
                return ArmActionResult.noChange();
                }
            }
        });

        ArmController.states.put(ArmState.SweepStart, new ArmAction() {
            private double ARM_ANGLE = -37;
            private double CARRIAGE_ANGLE = -5.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.PIDControlCarriage(CARRIAGE_ANGLE);
                // TODO: This needs to be tested, not sure if the pid will fail to keep both
                // close enough to their angles
                if (MainArm.PIDControlArm(ARM_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.SweepMiddleA, 250);
                }

                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.SweepMiddleA, new ArmAction() {
            private double ARM_ANGLE = -28;
            private double CARRIAGE_ANGLE = -22.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.spinIn();
                MainArm.PIDControlCarriage(CARRIAGE_ANGLE);
                // TODO: This needs to be tested, not sure if the pid will fail to keep both
                // close enough to their angles
                if (MainArm.PIDControlArm(ARM_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.SweepMiddleB, 700);
                }

                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.SweepMiddleB, new ArmAction() {
            private double ARM_ANGLE = -20;
            private double CARRIAGE_ANGLE = -20;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.spinIn();

                MainArm.PIDControlCarriage(CARRIAGE_ANGLE);

                // TODO: This needs to be tested, not sure if the pid will fail to keep both
                // close enough to their angles
                if (MainArm.PIDControlArm(ARM_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.SweepMiddleC, 1200);
                }
                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.SweepMiddleC, new ArmAction() {
            private double ARM_ANGLE = -23;
            private double CARRIAGE_ANGLE = -5.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.spinStop();
                MainArm.PIDControlCarriage(CARRIAGE_ANGLE);
                // TODO: This needs to be tested, not sure if the pid will fail to keep both
                // close enough to their angles
                if (MainArm.PIDControlArm(ARM_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.Finish, 1000);
                }
                return null;
            }
        });

        ArmController.states.put(ArmState.Finish, new ArmAction() {
            private double ARM_ANGLE = -7;
            private double CARRIAGE_ANGLE = -5.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.spinStop();
                // TODO: This needs to be tested, not sure if the pid will fail to keep both
                // close enough to their angles
                if (MainArm.PIDControlArm(ARM_ANGLE) && MainArm.PIDControlCarriage(CARRIAGE_ANGLE)) {
                    return ArmActionResult.changeTo(ArmState.MatchIdle);
                }
                return null;
            }
        });

        ArmController.states.put(ArmState.MatchIdle, new ArmAction() {
            private double ARM_ANGLE = -7;
            private double CARRIAGE_ANGLE = -5.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.spinStop();
                // TODO: This needs to be tested, not sure if the pid will fail to keep both
                // close enough to their angles
                if (MainArm.PIDControlArm(ARM_ANGLE) && MainArm.PIDControlCarriage(CARRIAGE_ANGLE)) {
                    return ArmActionResult.changeTo(ArmState.MatchIdle);
                }
                return null;
            }
        });

        // TODO: THIS
        ArmController.states.put(ArmState.ExtensionLow, new ArmAction() {
            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                double CARRIAGE_ANGLE = MainArm.getExtension(ArmState.Low);

                if (MainArm.PIDControlCarriage(CARRIAGE_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.Dropping, 1500);
                }
                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.ExtensionMedium, new ArmAction() {
            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                double CARRIAGE_ANGLE = MainArm.getExtension(ArmState.Medium);

                if (MainArm.PIDControlCarriage(CARRIAGE_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.Dropping, 1500);
                }
                return ArmActionResult.noChange();
            }
        });

        ArmController.states.put(ArmState.ExtensionHigh, new ArmAction() {
            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                double CARRIAGE_ANGLE = MainArm.getExtension(ArmState.High);

                if (MainArm.PIDControlCarriage(CARRIAGE_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.Dropping, 1500);
                }
                return ArmActionResult.noChange();
            }
        });

        // TODO: THIS
        ArmController.states.put(ArmState.Dropping, new ArmAction() {
            // private double CARRIAGE_ANGLE = -5.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {
                MainArm.spinOut();
                return ArmActionResult.changeToAfter(ArmState.Reel, 1000);

            }
        });

        ArmController.states.put(ArmState.Reel, new ArmAction() {
            private double CARRIAGE_ANGLE = -5.5;

            @Override
            public ArmActionResult run(ArmWrapper MainArm) {

                if (MainArm.PIDControlCarriage(CARRIAGE_ANGLE)) {
                    return ArmActionResult.changeToAfter(ArmState.MatchIdle, 2000);
                }
                return ArmActionResult.noChange();
            }
        });

    }
}