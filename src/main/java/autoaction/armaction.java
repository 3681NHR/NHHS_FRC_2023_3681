package autoaction;

import frc.robot.Robot;

import lib.interfaces.act;
import lib.interfaces.motorinterface;
import lib.motor.VictorWrapper;
import lib.motor.SparkWrapper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.XboxController;




public class armaction implements act { //I love java so much

    double Lset = 0;
    double Gset = 0;
    double process = 0;

    double time;
    double period;

    double c = 0; // shrimple and inefficient
    double d = 0;
    double output;
    double feedForward;

    private static final int ROTATING_ARM_CONTROLLER_CAN_ID = 6; // still doesnt work \o/ (it works) 3 weeks later
    private static final int ROTATING_ARM_ENCODER_PIN_A = 0; // TODO: Set this properly once it's plugged in
    private static final int ROTATING_ARM_ENCODER_PIN_B = 1; // currently set up to be the gripper carriage encoder

    private static final int GRIPPER_CARRIAGE_CONTROLLER_CAN_ID = 9;
    private static final int GRIPPER_CARRIAGE_ENCODER_PIN_A = 2; //tbd
    private static final int GRIPPER_CARRIAGE_ENCODER_PIN_B = 3; //tbd

    private static final int XBOX_CONTROLLER_USB_PORT2 = 1; // a little uneccessary tho

    private Encoder rotatingArmEncoder = new Encoder(ROTATING_ARM_ENCODER_PIN_A,
            ROTATING_ARM_ENCODER_PIN_B,
            false,
            CounterBase.EncodingType.k4X);
    private Encoder gripperCarriageEncoder = new Encoder(GRIPPER_CARRIAGE_ENCODER_PIN_A, 
            GRIPPER_CARRIAGE_ENCODER_PIN_B,
            false,
            CounterBase.EncodingType.k4X);
    XboxController xboxController2 = new XboxController(XBOX_CONTROLLER_USB_PORT2);
    motorinterface LiftAxisController = new SparkWrapper(ROTATING_ARM_CONTROLLER_CAN_ID, "Rotating Arm");
    CANSparkMax gripperCarriageController = new CANSparkMax(GRIPPER_CARRIAGE_CONTROLLER_CAN_ID,
            MotorType.kBrushed);
    
    private final SimpleMotorFeedforward rotatingArmFeedForward = new SimpleMotorFeedforward(0.38123, 0.07469);
    private final PIDController rotatingArmPIDController = new PIDController(2.0565, 0, .10);
    public double getTime() { 
      double period = System.currentTimeMillis(); //time at init
      return period; //theoretically
    }
    boolean active = true;

    final double angleM = 1/-360.0;

    public double Lsetpoint(boolean toggle) { //several hard stops in place
        if (toggle){
          if (process <= .2) {
            Lset = -30;
          } else if (process >= .2 && process <= .4) {
            Lset = -30;

          } else if (process >= .9) {
            Lset = 0;
          }
          return Lset;
        } else {
          return 0;
        }
    }

    public double Gsetpoint(boolean toggle) {
      if (toggle){
        if (process <= .2) {
          Gset = -30;
        } else if (process > .2 && process <= .4) {
          Gset = -30;

        } else if (process >= .9) {
          Gset = 0;
        }
        return Gset;

      } else {
        return 0;
      }
    }
    
    
    boolean something = false;
    @Override
    public void action(double num){
      time = System.currentTimeMillis();
      double difference = time - num;
      process = (1/difference)-(1/3000);
      rotateArmToPosition(-45);
      if (something == true) { 
        if (num+3000>=time){
          rotateArmToPosition(Lsetpoint(true));
        }
      } else {
        // return to analog control
      }
    }

    @Override 
    public boolean finished() {
      return rotatingArmEncoder.getStopped();
    }

    @Override 
    public void stop(){
      process = 0;
      active = false; 
      System.out.print("action stopped");
      
    }
    
    public void initializeRotatingArmEncoder() {
      double rotation = 360.0/2048.0; //the value to get accurate movement in degrees
      rotatingArmEncoder.setDistancePerPulse(rotation);
      rotatingArmEncoder.setSamplesToAverage(5);
      rotatingArmEncoder.setMinRate(0.05);
      rotatingArmEncoder.reset(); // GRAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH
  }

  public void rotateArmToPosition(double SET) {
      // Read 
      // Turn rotatingArm motor controller on yuh
      double multipler1 = -1;
      //LiftAxisController.set(ControlMode.Position, SET);
      double differencer = rotatingArmEncoder.getDistance() - SET;
      // god forbid this is wrong - if differencer is negative then it goes down right????
      // and positive means up??? !
      double speedcalc = ((differencer)/(Math.abs(differencer)));
      // actual distance = .1, distance = .45 then .1 - .45 is negative then you have to go
      setRotatingArmSpeed(speedcalc*multipler1, SET); // TODO: Make this speed a bit smarter
  }

  public void setRotatingArmSpeed(double rotatingArmSpeedMetersPerSecond, double setpoint) {
      final double feedForward = rotatingArmFeedForward.calculate(rotatingArmSpeedMetersPerSecond);
      double output =
           rotatingArmPIDController.calculate((rotatingArmEncoder.getDistance()), setpoint);
      System.out.println((output*.1+feedForward*.1)/10 + "PID !!!!");

      LiftAxisController.set(ControlMode.PercentOutput, (output*.1+feedForward*.1)/10); 
    }

  public void dashboardInfo() {
    SmartDashboard.putNumber("Encoder Distance", rotatingArmEncoder.getDistance());
    SmartDashboard.putNumber("Encoder Rate", rotatingArmEncoder.getRate());
    SmartDashboard.putNumber("Encoder Raw", rotatingArmEncoder.getRaw());
    SmartDashboard.putNumber("Carriage Encoder Distance", gripperCarriageEncoder.getDistance());
    SmartDashboard.putNumber("PID Value", output + feedForward);

  }

  public void calibrate() {
    rotatingArmEncoder.reset();
    gripperCarriageEncoder.reset();
  }

  public void analog(){
    if (xboxController2.getLeftBumper()) {
      c = 1; // if I give it some more voltage hrmrmm I can make it hold up but it will start
  } else if (xboxController2.getRightBumper()) {
      d = -1.0;
  } else {
      gripperCarriageController.set(0);
      c = 0;
      d = 0;
  }
    gripperCarriageController.set(.05 + c + d);
    //LiftAxisController.set(ControlMode.PercentOutput,((xboxController2.getLeftTriggerAxis() - xboxController2.getRightTriggerAxis()) * .2));
  }
  
  private void encoderstop() {
    rotatingArmEncoder.getStopped();
  }
}
