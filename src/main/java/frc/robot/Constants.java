// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final double ksVolts = 0.6666;
	public static final double kvVoltSecondsPerMeter = 8.32;
	public static final double kaVoltSecondsSquaredPerMeter = 0.0272;
	public static final double kPDriveVel = 0.0104;

	public static final Pose2d kInitialPose = new Pose2d(0.27355109961190166, 0.5559741267787839, new Rotation2d(0.015705514831139075));

	public static final double kTrackwidthMeters = 0.142072613;
	public static final double kMaxSpeedMetersPerSecond = 0.8;
	public static final double kMaxAccelerationMetersPerSecondSquared = 0.8;

	// Reasonable baseline values for a RAMSETE follower in units of meters and
	// seconds
	public static final double kRamseteB = 10;
	public static final double kRamseteZeta = 0.35;
}
