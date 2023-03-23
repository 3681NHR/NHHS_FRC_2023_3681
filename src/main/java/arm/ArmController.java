package arm;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import lib.interfaces.MotorInterface;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class ArmController {
    public enum ArmState {
        IDLE (0),
        HOME (1);

        private final int id;

        ArmState(int id) {
            this.id = id;
        }

        public int getId() {
            return this.id;
        }
    }

    private static final SimpleMotorFeedforward rotatingArmFeedForward = new SimpleMotorFeedforward(0.38123, 0.07469);
    private static final PIDController rotatingArmPIDController = new PIDController(2.0565, 0.0, 0.10);

    private interface ArmAction {
        public default ArmState run(Encoder armEncoder, Encoder carriageEncoder, MotorInterface armMotor, CANSparkMax carriageMotor) { return null; };
    }

    private Encoder armEncoder;
    private Encoder carriageEncoder;
    private MotorInterface armMotor;
    private CANSparkMax carriageMotor;

    private ArmState currentStateId;

    private static Map<ArmState, ArmAction> states;
    
    public ArmController(Encoder armEncoder, Encoder carriageEncoder, MotorInterface armMotor, CANSparkMax carriageMotor, ArmState defaultState) {
        requireNonNullParam(armEncoder, "armEncoder", "ArmController");
        requireNonNullParam(carriageEncoder, "carriageEncoder", "ArmController");
        requireNonNullParam(armMotor, "armMotor", "ArmController");
        requireNonNullParam(carriageMotor, "carriageMotor", "ArmController");

        this.armEncoder = armEncoder;
        this.carriageEncoder = carriageEncoder;
        this.armMotor = armMotor;
        this.carriageMotor = carriageMotor;

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

        ArmState newStateId = currentAction.run(this.armEncoder, this.carriageEncoder, this.armMotor, this.carriageMotor);
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
            private final double SPEED_SCALE = 0.05;
            private final double EPSILON = 0.01;

            @Override
            public ArmState run(Encoder armEncoder, Encoder carriageEncoder, MotorInterface armMotor, CANSparkMax carriageMotor) {
                double multipler1 = SPEED_SCALE * -1.0;
                double differencer = armEncoder.getDistance();

                double speed = differencer / Math.abs(differencer);
                speed *= multipler1;

                double rotatingArmSpeed = speed;
                double setpoint = 0.0;

                armMotor.set(ControlMode.PercentOutput, 0);

                final double feedForward = ArmController.rotatingArmFeedForward.calculate(rotatingArmSpeed);
                double output = ArmController.rotatingArmPIDController.calculate((armEncoder.getDistance()), setpoint);
                // System.out.println((output * 0.1 + feedForward * 0.1) / 10 + "PID !!!!");

                armMotor.set(ControlMode.PercentOutput, (output * 0.1 + feedForward * 0.1) / 10);

                if (Math.abs(differencer) <= EPSILON) {
                    return ArmState.IDLE;
                }

                return null;
            }
        });
    }
}

// public class ArmAction implements Act {
//     private static final int ROTATING_ARM_CONTROLLER_CAN_ID = 6; // NOTE: Still doesnt work \o/ (it works) 3 weeks later
//     private static final int ROTATING_ARM_ENCODER_PIN_A = 0; // TODO: Set this properly once it's plugged in
//     private static final int ROTATING_ARM_ENCODER_PIN_B = 1; // NOTE: Currently set up to be the gripper carriage encoder

//     private static final int GRIPPER_CARRIAGE_CONTROLLER_CAN_ID = 9;
//     private static final int GRIPPER_CARRIAGE_ENCODER_PIN_A = 2;
//     private static final int GRIPPER_CARRIAGE_ENCODER_PIN_B = 3;

//     private static final int XBOX_CONTROLLER_USB_PORT2 = 1; // NOTE: Is this necessary?

//     double Lset = 0.0;
//     double Gset = 0.0;
//     double process = 0.0;

//     double time;
//     double period;

//     double c = 0.0;
//     double d = 0.0;
//     double output;
//     double feedForward;

//     private Encoder rotatingArmEncoder = new Encoder(ROTATING_ARM_ENCODER_PIN_A,
//             ROTATING_ARM_ENCODER_PIN_B,
//             false,
//             CounterBase.EncodingType.k4X);
//     private Encoder gripperCarriageEncoder = new Encoder(GRIPPER_CARRIAGE_ENCODER_PIN_A,
//             GRIPPER_CARRIAGE_ENCODER_PIN_B,
//             false,
//             CounterBase.EncodingType.k4X);
//     XboxController xboxController2 = new XboxController(XBOX_CONTROLLER_USB_PORT2);
//     MotorInterface LiftAxisController = new SparkWrapper(ROTATING_ARM_CONTROLLER_CAN_ID, "Rotating Arm");
//     CANSparkMax gripperCarriageController = new CANSparkMax(GRIPPER_CARRIAGE_CONTROLLER_CAN_ID,
//             MotorType.kBrushed);

//     private final SimpleMotorFeedforward rotatingArmFeedForward = new SimpleMotorFeedforward(0.38123, 0.07469);
//     private final PIDController rotatingArmPIDController = new PIDController(2.0565, 0.0, 0.10);

//     public double getTime() {
//         double period = System.currentTimeMillis(); // time at init
//         return period; // theoretically
//     }

//     boolean active = true;

//     final double angleM = 1.0 / -360.0;

//     public double lSetPoint(boolean toggle) { // several hard stops in place
//         if (toggle) {
//             if (process <= 0.2) {
//                 Lset = -30.0;
//             } else if (process >= 0.2 && process <= 0.4) {
//                 Lset = -30.0;

//             } else if (process >= 0.9) {
//                 Lset = 0.0;
//             }
//             return Lset;
//         } else {
//             return 0.0;
//         }
//     }

//     public double gSetpoint(boolean toggle) {
//         if (toggle) {
//             if (process <= 0.2) {
//                 Gset = -30.0;
//             } else if (process > 0.2 && process <= 0.4) {
//                 Gset = -30.0;

//             } else if (process >= 0.9) {
//                 Gset = 0.0;
//             }
//             return Gset;

//         } else {
//             return 0.0;
//         }
//     }

//     boolean something = false;

//     @Override
//     public void action(double num) {
//         time = System.currentTimeMillis();
//         double difference = time - num;
//         process = (1.0 / difference) - (1.0 / 3000.0);
//         rotateArmToPosition(-45);
//         if (something == true) {
//             if (num + 3000.0 >= time) {
//                 rotateArmToPosition(lSetPoint(true));
//             }

//         } else {

//         }
//         if (finished()) {
//             return;
//         }
//     }

//     @Override
//     public boolean finished() {
//         return rotatingArmEncoder.getStopped();
//     }

//     @Override
//     public void stop() {
//         process = 0.0;
//         active = false;
//         System.out.print("action stopped");
//     }

//     public void initializeRotatingArmEncoder() {
//         double rotation = 360.0 / 2048.0; // the value to get accurate movement in degrees
//         rotatingArmEncoder.setDistancePerPulse(rotation);
//         rotatingArmEncoder.setSamplesToAverage(5);
//         rotatingArmEncoder.setMinRate(0.05);
//         rotatingArmEncoder.reset(); // GRAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH
//     }

//     public void rotateArmToPosition(double SET) {
//         double multipler1 = -1;
//         double differencer = rotatingArmEncoder.getDistance() - SET;

//         while (Math.abs(differencer) >= 3.1) {
//             differencer = rotatingArmEncoder.getDistance() - SET;

//             double speedcalc = ((differencer) / (Math.abs(differencer)));
//             setRotatingArmSpeed(speedcalc * multipler1, SET); // TODO: Make this speed a bit smarter
//         }
//         LiftAxisController.set(ControlMode.PercentOutput, 0);
//     }

//     // We'll use this in a little bit...
//     private class RotateThread extends Thread {

//         private double finalPosition = 0.0;

//         public RotateThread(double finalPosition) {
//             this.finalPosition = finalPosition;
//         }

//         @Override
//         public void run() {
//             double multipler1 = -1.0;
//             // LiftAxisController.set(ControlMode.Position, SET);
//             double differencer = rotatingArmEncoder.getDistance() - finalPosition;

//             while (Math.abs(differencer) >= 3.1) {
//                 differencer = rotatingArmEncoder.getDistance() - finalPosition;

//                 double speedcalc = ((differencer) / (Math.abs(differencer)));
//                 setRotatingArmSpeed(speedcalc * multipler1, finalPosition); // TODO: Make this speed a bit smarter
//             }
//             LiftAxisController.set(ControlMode.PercentOutput, 0.0);
//         }

//     }

//     public void setRotatingArmSpeed(double rotatingArmSpeed, double setpoint) {
//         final double feedForward = rotatingArmFeedForward.calculate(rotatingArmSpeed);
//         double output = rotatingArmPIDController.calculate((rotatingArmEncoder.getDistance()), setpoint);
//         System.out.println((output * .1 + feedForward * .1) / 10 + "PID !!!!");

//         LiftAxisController.set(ControlMode.PercentOutput, (output * .1 + feedForward * .1) / 10);
//     }

//     public void dashboardInfo() {
//         SmartDashboard.putNumber("Encoder Distance", rotatingArmEncoder.getDistance());
//         SmartDashboard.putNumber("Encoder Rate", rotatingArmEncoder.getRate());
//         SmartDashboard.putNumber("Encoder Raw", rotatingArmEncoder.getRaw());
//         SmartDashboard.putNumber("Carriage Encoder Distance", gripperCarriageEncoder.get());
//         SmartDashboard.putNumber("PID Value", output + feedForward);

//     }

//     public void calibrate() {
//         rotatingArmEncoder.reset();
//         gripperCarriageEncoder.reset();
//     }

//     public void analog() {
//         if (xboxController2.getLeftBumper()) {
//             c = 1; // if I give it some more voltage hrmrmm I can make it hold up but it will start
//         } else if (xboxController2.getRightBumper()) {
//             d = -1.0;
//         } else {
//             gripperCarriageController.set(0);
//             c = 0;
//             d = 0;
//         }
//         gripperCarriageController.set(.05 + c + d);
//         // LiftAxisController.set(ControlMode.PercentOutput,((xboxController2.getLeftTriggerAxis()
//         // - xboxController2.getRightTriggerAxis()) * .2));
//     }

//     private void encoderstop() {
//         rotatingArmEncoder.getStopped();
//     }
// }
