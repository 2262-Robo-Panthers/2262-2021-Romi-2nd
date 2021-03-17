// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.RomiDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();

	private final XboxController m_controller = new XboxController(0);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		final String trajectoryJSON = "paths/Auto.wpilib.json";
		Trajectory autoTrajectory = new Trajectory();
		try {
			autoTrajectory = TrajectoryUtil
				.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON));
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
		}

		final RamseteCommand ramseteCommand = new RamseteCommand(
			autoTrajectory,
			m_romiDrivetrain::getPose,
			new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
			new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
			new DifferentialDriveKinematics(Constants.kTrackwidthMeters),
			m_romiDrivetrain::getWheelSpeeds,
			m_romiDrivetrain.getLeftPID(),
			m_romiDrivetrain.getRightPID(),
			m_romiDrivetrain::tankDriveVolts,
			m_romiDrivetrain
		);

		m_romiDrivetrain.resetOdometry(autoTrajectory.getInitialPose());

		final Trajectory finalTrajectory = autoTrajectory;
		return new InstantCommand(() -> m_romiDrivetrain.resetOdometry(finalTrajectory.getInitialPose()), m_romiDrivetrain)
			.andThen(ramseteCommand)
			.andThen(m_romiDrivetrain::stopDrivetrain, m_romiDrivetrain);
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		m_romiDrivetrain.setDefaultCommand(
			new DriveCommand(
				m_romiDrivetrain,
				() -> -m_controller.getY(Hand.kLeft),
				() -> m_controller.getX(Hand.kRight),
				() -> m_controller.getBumper(Hand.kRight)));
	}

	public Command getTeleopResetCommand() {
		return new InstantCommand(() -> m_romiDrivetrain.resetOdometry(Constants.kTeleopStartPose), m_romiDrivetrain);
	}
}
