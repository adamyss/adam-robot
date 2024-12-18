package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.util.Units;
// import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
  public static final int kBottomCanId = 24;
  public static final int kTopCanId = 25;

  // MEASUREMENTS
  // public static final double kBottomWheelDiameter = Units.inchesToMeters(3.0);
  // // meters
  // public static final double kTopWheelDiameter = Units.inchesToMeters(4.0); //
  // meters

  // ---
  private CANSparkMax m_bottom;
  private CANSparkMax m_top;

  private RelativeEncoder m_bottomEncoder;
  private RelativeEncoder m_topEncoder;

  // private final MutableMeasure<Voltage> m_appliedVoltage =
  // mutable(Volts.of(0));
  // private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // private final MutableMeasure<Velocity<Distance>> m_velocity =
  // mutable(MetersPerSecond.of(0));

  // SysIdRoutine routine;

  public Shooter() {
    m_bottom = new CANSparkMax(kBottomCanId, MotorType.kBrushless);
    m_top = new CANSparkMax(kTopCanId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. Useful in case a SPARK MAX is swapped out.
    m_bottom.restoreFactoryDefaults();
    m_top.restoreFactoryDefaults();

    m_bottom.setInverted(true);
    m_top.setInverted(false);

    // Setup encoders and PID controllers
    m_bottomEncoder = m_bottom.getEncoder();
    m_topEncoder = m_top.getEncoder();

    setCoast();
    m_bottom.setSmartCurrentLimit(80);
    m_top.setSmartCurrentLimit(80);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_bottom.burnFlash();
    m_top.burnFlash();

    m_bottomEncoder.setPosition(0);
    m_topEncoder.setPosition(0);

    // Creates a SysIdRoutine
    // routine = new SysIdRoutine(
    // new SysIdRoutine.Config(),
    // new SysIdRoutine.Mechanism(this::voltageShoot,
    // log -> {
    // log.motor("shoot-top")
    // .voltage(
    // m_appliedVoltage.mut_replace(
    // m_top.get() * RobotController.getBatteryVoltage(), Volts))
    // .linearPosition(m_distance.mut_replace(m_top.getEncoder().getPosition(),
    // Meters))
    // .linearVelocity(
    // m_velocity.mut_replace(m_top.getEncoder().getVelocity(), MetersPerSecond));
    // log.motor("shoot-bottom")
    // .voltage(
    // m_appliedVoltage.mut_replace(
    // m_bottom.get() * RobotController.getBatteryVoltage(), Volts))
    // .linearPosition(m_distance.mut_replace(m_bottom.getEncoder().getPosition(),
    // Meters))
    // .linearVelocity(
    // m_velocity.mut_replace(m_bottom.getEncoder().getVelocity(),
    // MetersPerSecond));
    // },
    // this));
  }

  // private void voltageShoot(Measure<Voltage> volts) {
  // m_bottom.setVoltage(volts.in(Volts));
  // m_top.setVoltage(volts.in(Volts));
  // }

  // Set the shooter motors to brake
  public void setBrake() {
    m_bottom.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_top.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  // Set the shooter motors to coast
  public void setCoast() {
    m_bottom.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_top.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public double getCurrent() {
    return m_top.getOutputCurrent();
  }

  public void set(double speed) {
    SmartDashboard.putNumber("Shooter Speed", speed);

    m_bottom.set(speed);
    m_top.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Top Encoder Position", m_topEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Top Encoder Velocity", m_topEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Top Temp", m_top.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Top Current", m_top.getOutputCurrent());

    SmartDashboard.putNumber("Shooter Bottom Encoder Position", m_bottomEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Bottom Encoder Velocity", m_bottomEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Bottom Temp", m_bottom.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Bottom Current", m_bottom.getOutputCurrent());
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  // return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  // return routine.dynamic(direction);
  // }
}
