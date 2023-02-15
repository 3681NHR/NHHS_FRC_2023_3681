package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SparkWrapper implements MotorController {
  private final CANSparkMax inner;
  private boolean inverted;
  SparkWrapper(int device) {
      inner = new CANSparkMax(device, MotorType.kBrushless);
  }

  @Override
  public void set(double speed) {
      if (inverted) {
          speed *= -1;
      }
      inner.set(speed);
  }

  @Override
  public double get() {
      return inner.get();
  }

  @Override
  public void setInverted(boolean isInverted) {
      inverted = isInverted;
  }

  @Override
  public boolean getInverted() {
      return inverted;
  }

  @Override
  public void disable() {}

  @Override
  public void stopMotor() {}
}

