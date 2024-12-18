// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import necessary classes and subsystems
import frc.robot.Constants.OperatorConstants; // Constants for operator settings
import frc.robot.commands.DefaultDrive; // Default driving command
import frc.robot.subsystems.Indexing; // Indexing subsystem for handling objects
import frc.robot.subsystems.Intake; // Intake subsystem for collecting objects
import frc.robot.subsystems.Shooter; // Shooter subsystem for launching objects
import frc.robot.subsystems.Spin;
import frc.robot.subsystems.SwerveSubsystem; // Swerve drive subsystem for movement

import frc.robot.subsystems.Wrist; // Wrist subsystem for aiming
import edu.wpi.first.wpilibj2.command.Command; // Command base class
import edu.wpi.first.wpilibj2.command.InstantCommand; // Command that runs once and immediately finishes
import edu.wpi.first.wpilibj2.command.StartEndCommand; // Command that runs a start action while active
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; // Xbox controller support
import frc.robot.commands.AutonCommand; // Autonomous command
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // SmartDashboard
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup; // Parallel command group
import edu.wpi.first.wpilibj2.command.WaitCommand; // Wait command

/**
 * The RobotContainer class is responsible for declaring the robot's subsystems,
 * commands, and configuring the input bindings for the controller.
 */
public class RobotContainer {
  // Define robot subsystems as public static final to allow access throughout the
  // class
  public static final SwerveSubsystem m_Swerb = new SwerveSubsystem();
  public static final Spin m_spin = new Spin(m_Swerb);
  public static final Intake intake = new Intake();
  public static final Indexing indexing = new Indexing();
  public static final Shooter shooter = new Shooter();
  public static final Wrist wrist = new Wrist();
  public static final AutonCommand m_autonCommand = new AutonCommand(m_Swerb, intake, indexing, shooter, wrist);

  // Create an Xbox controller instance to handle driver input (0 is the port
  // number)
  public static CommandXboxController driverController = new CommandXboxController(0);

  /**
   * Constructor for RobotContainer. This is called when the robot is initialized.
   * It sets up the command bindings for the robot's controls.
   */
  public RobotContainer() {
    // Call the method to configure the input bindings
    configureBindings();
  }

  /**
   * This method maps controller inputs to commands that control the robot's
   * subsystems.
   * It defines the behavior for each button press and its associated actions.
   */
  private void configureBindings() {
    // Set the default command for the swerve drive subsystem
    m_Swerb.setDefaultCommand(new DefaultDrive());

    // Configure the x button button to reset the robot's yaw (orientation)

    //driverController.x().onTrue(new InstantCommand(() -> m_Swerb.zeroYaw())); // Resets the yaw when D-Pad Up is pressed
    driverController.x().onTrue(m_spin.spin());
     driverController.x().onFalse(m_spin.noSpin());

  

    // Intake control: when left bumper is pressed
    driverController.leftBumper()
        .whileTrue(new StartEndCommand(
            () -> {
              intake.set(1); // Sets intake motor to full forward (1.0)
              indexing.set(1); // Sets indexing motor to full forward (1.0)
            },
            () -> {
              intake.set(0); // Stops the intake motor (0)
              indexing.set(0); // Stops the indexing motor (0)
            },
            intake, indexing // Specifies the subsystems to be affected
        ));

    // Creating a StartEndCommand that starts the shooter and stops it when the
    // button is releas

    // Shooting control: when right bumper is pressed
    driverController.rightBumper().whileTrue(new ParallelCommandGroup(
        new StartEndCommand(
            () -> {
              shooter.set(-1); // starts the shooter
            },
            () -> {
              shooter.set(0); // stops the shooter
            },
            shooter),
        new WaitCommand(1.5).andThen(
            new StartEndCommand(
                () -> {
                  // shooter.set(-1); // continues shooting
                  indexing.set(1); // starts the indexing
                },
                () -> {
                  // shooter.set(0); // stops the shooter
                  indexing.set(0); // stops the indexing
                },
                indexing).withTimeout(1) // retain maximum power
        )));

    // Moving the indexer up slowly to fine tune the shot
    driverController.a()
        .whileTrue(new StartEndCommand(
            () -> {
              indexing.set(0.2);
            },
            () -> {
              indexing.set(0);
            }));

    // Floor outtake
    driverController.b()
        .whileTrue(new StartEndCommand(
            () -> {
              intake.set(-0.8);
            },
            () -> {
              intake.set(0);
            }));

    // Front intake
    driverController.y()
        .whileTrue(new StartEndCommand(
            () -> {
              shooter.set(.7); // Sets shooter motor to moderate forward (0.4)
              indexing.set(-.6); // Sets indexing motor to moderate reverse (-0.6)
            },
            () -> {
              shooter.set(0); // Stops the shooter motor (0)
              indexing.set(0); // Stops the indexing motor (0)
            },
            shooter, indexing // Specifies the subsystems to be affected
        ));

    // The 0.2 is the threshold / deadzone for the trigger
    driverController.rightTrigger(0.2).whileTrue(new InstantCommand(() -> {
      wrist.wristUp(0.15);
    })).whileFalse(new InstantCommand(() -> {
      wrist.wristUp(0);
    }));

    driverController.leftTrigger(0.2).whileTrue(new InstantCommand(() -> {
      wrist.wristDown(0.15);
    })).whileFalse(new InstantCommand(() -> {
      wrist.wristDown(0);
    }));

    // Wind-up shooting mechanism: when X button is held
    // driverController.x()
    // .whileTrue(new StartEndCommand(
    // () -> {
    // shooter.set(-1); // Sets shooter motor to full reverse (-1.0)
    // },
    // () -> {
    // shooter.set(0); // Stops the shooter motor (0)
    // },
    // shooter // Specifies the shooter subsystem
    // ));

    // Uncommented stop all command (can be used to stop everything)
    // driverController.leftBumper()
    // .onTrue(new InstantCommand(() -> {
    // intake.set(0); // Stops the intake motor
    // indexing.set(0); // Stops the indexing motor
    // shooter.set(0); // Stops the shooter motor
    // }, intake, indexing, shooter));

    // Uncommented wrist controls for aiming
    // driverController.leftBumper().onTrue(
    // wrist.incrementUp() // Increments the wrist position up
    // );
    // driverController.rightBumper().onTrue(
    // wrist.incrementDown() // Increments the wrist position down
    // );
  }

  /**
   * This method provides the command to run during the autonomous phase.
   *
   * @return The command to execute in autonomous mode.
   */
  public AutonCommand getAutonomousCommand() {
    return m_autonCommand;
  }
}
