package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral_arm extends SubsystemBase {
  SparkMax motor;
  CANcoder encoder = new CANcoder(25);
  ProfiledPIDController pid;
  private double m_targetSpeed = 0.0;
  StatusSignal<Angle> angleValue = encoder.getAbsolutePosition();

  public Coral_arm() {
    motor = new SparkMax(22, MotorType.kBrushed);
    SparkMaxConfig motorconfig = new SparkMaxConfig();
    motorconfig.inverted(true);
    motor.configure(motorconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = 0.368652;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder.getConfigurator().apply(config);
    pid = new ProfiledPIDController(180, 0, 0, new Constraints(0.1, 2.5));
    BaseStatusSignal.setUpdateFrequencyForAll(50, angleValue);
  }

  public void periodic() {
    BaseStatusSignal.refreshAll(angleValue);
    // setTargetSpeed(m_targetSpeed);
    // motor.set(m_targetSpeed);
    // This method will be called once per scheduler run
  }

  public void setTargetPosition(double rotations) {
    double voltage = pid.calculate(angleValue.getValue().in(Rotations), rotations);
    System.out.println(voltage);
    motor.setVoltage(voltage);
  }

  public void stop() {
    motor.stopMotor();
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
