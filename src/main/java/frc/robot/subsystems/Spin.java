// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.Instant;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.SwerveSubsystem;


public class Spin extends SubsystemBase {
       final double omegaRadiansPerSecond = Math.PI/6;
       
       
        private final SwerveSubsystem swerve;
  /** Creates a new ExampleSubsystem. */
  public Spin(SwerveSubsystem swerve) {
    this.swerve = swerve;

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */


  public InstantCommand spin() {
    return new InstantCommand(() -> swerve.drive(new ChassisSpeeds(0, 0, omegaRadiansPerSecond))); // Set timeout for 0.5 seconds
   
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

 
  public InstantCommand noSpin(){
 return new InstantCommand(() -> swerve.drive(new ChassisSpeeds(0,
0, 0)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}