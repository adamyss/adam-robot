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
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Indexing extends SubsystemBase {
  // CAN IDs
  public static final int kLeftCanId = 21;
  public static final int kRightCanId = 22;

  // MEASUREMENTS
  public static final double kWheelDiameter = Units.inchesToMeters(3.0); // meters
  public static final double kGearRatio = 1.0 / 16.0; // 16:1 gear ratio

  // ---
  private boolean loaded = false;

  private CANSparkMax m_left;
  private CANSparkMax m_right;

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  // AnalogInput m_beamBreakBottom;
  // AnalogInput m_beamBreakTop;

  double beamBreakThreshold = 10.0; // idk what unit

  // private final MutableMeasure<Voltage> m_appliedVoltage =
  // mutable(Volts.of(0));
  // private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // private final MutableMeasure<Velocity<Distance>> m_velocity =
  // mutable(MetersPerSecond.of(0));

  // SysIdRoutine routine;

  public Indexing() {
    m_left = new CANSparkMax(kLeftCanId, MotorType.kBrushless);
    m_right = new CANSparkMax(kRightCanId, MotorType.kBrushless);

    // m_beamBreakBottom = new AnalogInput(0);
    // m_beamBreakTop = new AnalogInput(1);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. Useful in case a SPARK MAX is swapped out.
    m_left.restoreFactoryDefaults();
    m_right.restoreFactoryDefaults();

    m_left.setInverted(false);
    m_right.setInverted(true);

    // Setup encoders and PID controllers
    m_leftEncoder = m_left.getEncoder();
    m_rightEncoder = m_right.getEncoder();

    setCoast();
    m_left.setSmartCurrentLimit(50);
    m_right.setSmartCurrentLimit(50);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_left.burnFlash();
    m_right.burnFlash();

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    // Additional initialization stuff here if needed
    setCoast();

    // // Creates a SysIdRoutine
    // routine = new SysIdRoutine(
    // new SysIdRoutine.Config(),
    // new SysIdRoutine.Mechanism(this::voltageIndexing,
    // log -> {
    // log.motor("indexing")
    // .voltage(
    // m_appliedVoltage.mut_replace(
    // m_left.get() * RobotController.getBatteryVoltage(), Volts))
    // .linearPosition(m_distance.mut_replace(m_left.getEncoder().getPosition(),
    // Meters))
    // .linearVelocity(
    // m_velocity.mut_replace(m_left.getEncoder().getVelocity(), MetersPerSecond));
    // },
    // this
    // ));
  }

  // private void voltageIndexing(Measure<Voltage> volts){
  // m_left.setVoltage(volts.in(Volts));
  // m_right.setVoltage(-volts.in(Volts));
  // }

  public double getCurrent() {
    return m_right.getOutputCurrent();
  }

  public double getVelocity() {
    return m_rightEncoder.getVelocity();
  }

  public void setBrake() {
    m_left.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_right.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setCoast() {
    m_left.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_right.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void set(double speed) {
    SmartDashboard.putNumber("Indexing Speed", speed);

    m_left.set(speed);
    m_right.set(speed);
  }

  public void setLoaded(boolean b) {
    loaded = b;
    SmartDashboard.putString("State", loaded ? "loaded" : "empty");
  }

  public boolean isLoaded() {
    SmartDashboard.putString("State", loaded ? "loaded" : "empty");
    return loaded;
  }

  public void stop() {
    SmartDashboard.putString("Indexing State", "stopped");

    // m_right.set(0);
    // m_left.set(0);

    m_left.set(0);
    m_right.set(0);
  }

  // public boolean isTopSensorSensing() {
  // return m_beamBreakTop.getVoltage() > beamBreakThreshold;
  // }

  // public boolean isBottomSensorSensing() {
  // return m_beamBreakBottom.getVoltage() > beamBreakThreshold;
  // }

  public void periodic() {
    SmartDashboard.putNumber("Indexing Left Encoder Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Indexing Left Encoder Velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Indexing Left Temp", m_left.getMotorTemperature());

    SmartDashboard.putNumber("Indexing Right Encoder Position", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Indexing Right Encoder Velocity", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Indexing Right Temp", m_right.getMotorTemperature());

    SmartDashboard.putNumber("Indexing Current", getCurrent());

    // SmartDashboard.putNumber("Bottom Photosensor Voltage",
    // m_beamBreakBottom.getVoltage());
    // SmartDashboard.putBoolean("Bottom Photosensor Detected",
    // m_beamBreakBottom.getVoltage() > beamBreakThreshold);

    // SmartDashboard.putNumber("Top Photosensor Voltage",
    // m_beamBreakTop.getVoltage());
    // SmartDashboard.putBoolean("Top Photosensor Detected",
    // m_beamBreakTop.getVoltage() > beamBreakThreshold);
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  // return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  // return routine.dynamic(direction);
  // }
}