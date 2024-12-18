// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** A robot wrist subsystem that moves with a motion profile. */
public class Wrist extends TrapezoidProfileSubsystem {
  private final CANSparkMax m_motor;
  private final AbsoluteEncoder m_encoder;
  private final SparkPIDController m_pidController;

  // CAN IDs
  static final int kArmCanId = 23;

  // THROUGHBORE ENCODER
  static final int kCountsPerRev = 8192;

  // These are fake gains; in actuality these must be determined individually for
  // each robot
  static final double kSVolts = 1;
  static final double kGVolts = 1;
  static final double kVVoltSecondPerRad = 0.5;
  static final double kAVoltSecondSquaredPerRad = 0.1;

  static final double kMaxVelocityRadPerSecond = 3;
  static final double kMaxAccelerationRadPerSecSquared = 1;

  // The offset of the arm from the horizontal in its neutral position,
  // measured from the horizontal
  static final double kWristOffsetRads = 0.5;

  // ------

  private final double defaultPosition = Math.PI / 6; // radians
  private double currentPosition = Math.PI / 6; // radians

  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      kSVolts, kGVolts,
      kVVoltSecondPerRad, kAVoltSecondSquaredPerRad);

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  SysIdRoutine routine;

  double threshold = .4;

  /** Create a new Wrist. */
  public Wrist() {
    super(new TrapezoidProfile.Constraints(kMaxVelocityRadPerSecond, kMaxAccelerationRadPerSecSquared),
        kWristOffsetRads);

    m_motor = new CANSparkMax(kArmCanId, MotorType.kBrushless);

    // Factory reset, so we get the SPARK MAX to a known state before configuring
    // them. Useful in case a SPARK MAX is swapped out.
    m_motor.restoreFactoryDefaults();

    m_encoder = m_motor.getAbsoluteEncoder();
    m_pidController = m_motor.getPIDController();

    // m_encoder.setPositionConversionFactor(2*Math.PI);
    // m_encoder.setVelocityConversionFactor(2*Math.PI);

    m_encoder.setZeroOffset(0.4);
    m_encoder.setInverted(true);

    // m_motor.setSmartCurrentLimit(MotorContants.kMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_motor.burnFlash();

    // m_encoder.setPosition(0);

    m_pidController.setP(.01, 0);
    m_pidController.setI(0, 0);
    m_pidController.setD(0, 0);

    m_pidController.setFeedbackDevice(m_encoder);

    m_motor.setSmartCurrentLimit(30);

    // setWristGoalDefaultCommand();

    // Creates a SysIdRoutine
    routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(this::voltageAim,
            log -> {
              log.motor("aim")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          m_motor.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(m_encoder.getPosition(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(m_encoder.getVelocity(), MetersPerSecond));
            },
            this));
  }

  private void voltageAim(Measure<Voltage> volts) {
    m_motor.setVoltage(volts.in(Volts));
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    // Set the setpoint for the PID controller
    m_pidController.setReference(setpoint.position, CANSparkBase.ControlType.kPosition, 0, feedforward / 12.0);
  }

  /** @param kWristOffsetRads in radians */
  public Command setWristGoalCommand(double kWristOffsetRads) {
    currentPosition = kWristOffsetRads;
    return Commands.runOnce(() -> setSafeGoal(currentPosition), this);
  }

  public Command setWristGoalDefaultCommand() {
    currentPosition = defaultPosition;
    return Commands.runOnce(() -> setSafeGoal(currentPosition), this);
  }

  public Command incrementUp() {
    currentPosition += Units.degreesToRadians(1);
    return Commands.runOnce(() -> setSafeGoal(currentPosition), this);
  }

  public Command incrementDown() {
    currentPosition -= Units.degreesToRadians(1);
    return Commands.runOnce(() -> setSafeGoal(currentPosition), this);
  }

  public void wristUp(double speed) {
    m_motor.set(speed);
  }

  public void wristDown(double speed) {
    m_motor.set(-speed);
  }

  /** @param angle in radians */
  public void setSafeGoal(double angle) {
    if (angle < 0 || angle > Units.rotationsToRadians(0.0966))
      angle = Math.PI / 6; // set safe if out of bounds
    setGoal(Units.radiansToRotations(angle));
    SmartDashboard.putNumber("Wrist angle", angle);
  }

  // /**
  // * corrects angle
  // * @return corrected angle in degrees
  // */
  // public double getAngle() {
  // double angle = m_encoder.getPosition();
  // if (angle < threshold) angle += 1.0;
  // angle *= 360;
  // return 90 - angle;
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Raw Angle", m_encoder.getPosition());
    // SmartDashboard.putNumber("Wrist Corrected Angle", getAngle());
    // SmartDashboard.putNumber("Wrist Encoder Velocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("Wrist Temp", m_motor.getMotorTemperature());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
