package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake_Algea extends SubsystemBase {
  SparkMax motor;
  private double m_targetSpeed = 0.0;

  public Intake_Algea() {
    motor = new SparkMax(21, MotorType.kBrushed);
  }

  @Override
  public void periodic() {
    motor.set(m_targetSpeed);
  }

  public void setTargetSpeed(double targetSpeed) {
    m_targetSpeed = targetSpeed;
  }

  public double getTargetSpeed() {
    return m_targetSpeed;
  }
}
