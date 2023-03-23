package arm;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class ArmController {
    public enum ArmState {
        IDLE (0),
        HOME (1),
        LOW (2),
        MEDIUM(3),
        HIGH(4),
        SWEEPSTART(5),
        SWEEPMIDDLE(6),
        SWEEPFINISH(7);

        private final int id;

        ArmState(int id) {
            this.id = id;
        }

        public int getId() {
            return this.id;
        }
    }

    static final SimpleMotorFeedforward rotatingArmFeedForward = new SimpleMotorFeedforward(0.38123, 0.07469);
    static final PIDController rotatingArmPIDController = new PIDController(2.0565, 0.0, 0.10);

    private interface ArmAction {
        public default ArmState run(ArmWrapper MainArm) { return null; };
    }

    private ArmWrapper MainArm;

    private ArmState currentStateId;

    private static Map<ArmState, ArmAction> states;
    /**
     * 
     * @param MainArm ArmWrapper object
     * @param defaultState null
     */
    public ArmController(ArmWrapper MainArm, ArmState defaultState) {
        requireNonNullParam(MainArm, "MainArm", "ArmController");
 

        this.MainArm = MainArm;
       

        if (defaultState == null) {
            this.currentStateId = ArmState.IDLE;
        } else {
            this.currentStateId = defaultState;
        }
    }

    public void runPeriodic() {
        ArmAction currentAction = ArmController.states.get(this.currentStateId);
        if (currentAction == null) {
            throw new RuntimeException("Attempted to run action with no corresponding action id");
        }

        ArmState newStateId = currentAction.run(this.MainArm);
        if (newStateId != null) {
            this.currentStateId = newStateId;
        }
    }

    public ArmState getState() {
        return currentStateId;
    }

    public void setState(ArmState state) {
        this.currentStateId = state;
    }

    static {
        ArmController.states.put(ArmState.IDLE, new ArmAction() {});

        ArmController.states.put(ArmState.HOME, new ArmAction() {
            private final double EPSILON = 0.1;
            private final double HOME_POINT = 0;

            @Override
            public ArmState run(ArmWrapper MainArm) {
                double differencer = MainArm.getAngleArm() - HOME_POINT;

                MainArm.PIDControlArm(HOME_POINT);

                if (Math.abs(differencer) <= EPSILON) {
                    return ArmState.IDLE;
                }
                return null;
            }
        });

        ArmController.states.put(ArmState.LOW, new ArmAction(){
            private final double EPSILON = 0.1;
            private final double LOW_POINT = -20;

            @Override
            public ArmState run(ArmWrapper MainArm) {
                double differencer = MainArm.getAngleArm() - LOW_POINT;

                MainArm.PIDControlArm(LOW_POINT);

                if (Math.abs(differencer) <= EPSILON) {
                    return ArmState.IDLE;
                }
                return null;
            }
        });

        ArmController.states.put(ArmState.MEDIUM, new ArmAction(){
            private final double EPSILON = 0.1;
            private final double MEDIUM_POINT = -50;

            @Override
            public ArmState run(ArmWrapper MainArm) {
                double differencer = MainArm.getAngleArm() - MEDIUM_POINT;

                MainArm.PIDControlArm(MEDIUM_POINT);

                if (Math.abs(differencer) <= EPSILON) {
                    return ArmState.IDLE;
                }
                return null;
            }
        });

    }
}