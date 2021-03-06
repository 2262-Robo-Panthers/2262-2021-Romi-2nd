// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveCommand extends CommandBase {

	private final RomiDrivetrain m_drivetrain;

	private final DoubleSupplier m_speedSupplier;
	private final DoubleSupplier m_rotateSupplier;
	private final BooleanSupplier m_quickTurnSupplier;

	/** Creates a new DriveCommand. */
	public DriveCommand(RomiDrivetrain drivetrain, DoubleSupplier speedSupplier, DoubleSupplier rotateSupplier,
			BooleanSupplier quickTurnSupplier) {
		m_drivetrain = drivetrain;
		m_speedSupplier = speedSupplier;
		m_rotateSupplier = rotateSupplier;
		m_quickTurnSupplier = quickTurnSupplier;
		addRequirements(m_drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// final double throttle = m_speedSupplier.getAsDouble();
		// final double wheel = m_rotateSupplier.getAsDouble() * 0.05;
		// final double kScrubFactor = 1.0469745223;
		// final double delta_v = Units.metersToInches(Constants.kTrackwidthMeters) * wheel / (2 * kScrubFactor);
		// final double leftVel = throttle - delta_v;
		// final double rightVel = throttle + delta_v;
		// final double scaling_factor = Math.max(1.0, Math.max(Math.abs(leftVel), Math.abs(rightVel)));
		// m_drivetrain.tankDriveVolts(leftVel / scaling_factor * 12, rightVel / scaling_factor * 12);
		m_drivetrain.curvatureDrive(
			m_speedSupplier.getAsDouble(), m_rotateSupplier.getAsDouble(), m_quickTurnSupplier.getAsBoolean());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
