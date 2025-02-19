package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  CANcoder encoder;
  SparkMax motor;
  private double m_targetSpeed = 0.0;

  public Arm() {
    motor = new SparkMax(22, MotorType.kBrushed);
  }

  public void periodic() {
    // setTargetSpeed(m_targetSpeed);
    motor.set(m_targetSpeed);
    // This method will be called once per scheduler run
  }

  public void setTargetSpeed(double targetSpeed) {

    m_targetSpeed = targetSpeed;
  }

  public double getTargetSpeed() {
    return m_targetSpeed;
  }

  public double getMotorSpeed() {
    return motor.getEncoder().getVelocity();
  }
}
