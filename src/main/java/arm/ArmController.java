package arm;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class ArmController {

    public enum ArmState {
        IDLE (0),
        HOME (1),
        LOW (2),
        MEDIUM(3),
        HIGH(4),
        SWEEPSTART(5),
        SWEEPMIDDLE1(6),
        SWEEPMIDDLE2(7),
        SWEEPMIDDLE3(8),
        SWEEPFINISH(9),
        RECALIBRATE(10);

        private final int id;

        ArmState(int id) {
            this.id = id;
        }

        public int getId() {
            return this.id;
        }
    }

    private interface ArmAction {
        public default ArmState run(ArmWrapper MainArm) { return null; };
    }

    private ArmWrapper MainArm;

    private ArmState currentStateId;

    private static Map<ArmState, ArmAction> states = new HashMap<>();
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
        ArmController.states.put(ArmState.IDLE, new ArmAction() {
            @Override
            public ArmState run(ArmWrapper MainArm) {
                MainArm.analogArm(0);
                return ArmState.IDLE;
            }
        });

        ArmController.states.put(ArmState.HOME, new ArmAction() {
            private final double EPSILON = 1;
            private final double HOME_POINT = 0;

            @Override
            public ArmState run(ArmWrapper MainArm) {
                double differencer = MainArm.getAngleArm() - HOME_POINT;
                if (Math.abs(differencer) <= EPSILON) {
                MainArm.PIDControlArm(HOME_POINT, false);
                } else {
                MainArm.PIDControlArm(HOME_POINT, true);
                }
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
                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlArm(LOW_POINT, false);
                    } else {
                    MainArm.PIDControlArm(LOW_POINT, true);
                    }

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

                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlArm(MEDIUM_POINT, false);
                    } else {
                    MainArm.PIDControlArm(MEDIUM_POINT, true);
                    }

                if (Math.abs(differencer) <= EPSILON) {
                    return ArmState.IDLE;
                }
                return null;
            }
        });

        ArmController.states.put(ArmState.RECALIBRATE, new ArmAction(){
            private final double EPSILON = 0.1;

            @Override
            public ArmState run(ArmWrapper MainArm) {
                double differencer = MainArm.getAngleArm() - 0;

                MainArm.analogArm(-.1);

                if (Math.abs(differencer) <= EPSILON) {
                    return ArmState.IDLE;
                }
                return null;
            }
        });

        ArmController.states.put(ArmState.SWEEPSTART, new ArmAction(){
            private final double EPSILON = 0.1;
            private double POINT = -28;
            private double GPOINT = -5.5;

            @Override
            public ArmState run(ArmWrapper MainArm) {
                double differencer = MainArm.getAngleArm() - POINT;

                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlArm(POINT, false);
                    } else {
                    MainArm.PIDControlArm(POINT, true);
                    }
                if (Math.abs(differencer) <= EPSILON) {
                    return ArmState.IDLE;
                }
                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlCarriage(GPOINT, false);
                    } else {
                    MainArm.PIDControlCarriage(GPOINT, true);
                    }
                return null;
            }
        });

        ArmController.states.put(ArmState.SWEEPMIDDLE1, new ArmAction(){
            private final double EPSILON = 0.1;
            private double POINT = -29;
            private double GPOINT = -21.5;

            @Override
            public ArmState run(ArmWrapper MainArm) {
                double differencer = MainArm.getAngleArm() - POINT;

                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlArm(POINT, false);
                    } else {
                    MainArm.PIDControlArm(POINT, true);
                    }
                if (Math.abs(differencer) <= EPSILON) {
                    return ArmState.IDLE;
                }
                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlCarriage(GPOINT, false);
                    } else {
                    MainArm.PIDControlCarriage(GPOINT, true);
                    }
                return null;
            }
        });

        ArmController.states.put(ArmState.SWEEPMIDDLE2, new ArmAction(){
            private final double EPSILON = 0.1;
            private double POINT = -23;
            private double GPOINT = -18;
            @Override
            public ArmState run(ArmWrapper MainArm) {
                double differencer = MainArm.getAngleArm() - POINT;

                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlArm(POINT, false);
                    } else {
                    MainArm.PIDControlArm(POINT, true);
                    }
                
                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlCarriage(GPOINT, false);
                    } else {
                    MainArm.PIDControlCarriage(GPOINT, true);
                    }
                if (Math.abs(differencer) <= EPSILON) {
                    return ArmState.IDLE;
                }
                return null;
            }
        });

        ArmController.states.put(ArmState.SWEEPMIDDLE3, new ArmAction(){
            private final double EPSILON = 0.1;
            private double POINT = -23;
            private double GPOINT = -5.5;
            @Override
            public ArmState run(ArmWrapper MainArm) {
                double differencer = MainArm.getAngleArm() - POINT;

                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlArm(POINT, false);
                    } else {
                    MainArm.PIDControlArm(POINT, true);
                    }
                
                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlCarriage(GPOINT, false);
                    } else {
                    MainArm.PIDControlCarriage(GPOINT, true);
                    }
                if (Math.abs(differencer) <= EPSILON) {
                    return ArmState.IDLE;
                }
                return null;
            }
        });

        ArmController.states.put(ArmState.SWEEPFINISH, new ArmAction(){
            private final double EPSILON = 0.001;
            private double POINT = -7;
            private double GPOINT = -5.5;

            @Override
            public ArmState run(ArmWrapper MainArm) {
                double differencer = MainArm.getAngleArm() - POINT;

                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlArm(POINT, false);
                    } else {
                    MainArm.PIDControlArm(POINT, true);
                    }
                
                if (Math.abs(differencer) <= EPSILON) {
                    MainArm.PIDControlCarriage(GPOINT, false);
                    } else {
                    MainArm.PIDControlCarriage(GPOINT, true);
                    }
             
                if (Math.abs(differencer) <= EPSILON) {
                    return ArmState.IDLE;
                }
                return null;
            }
        });
        
    }
}