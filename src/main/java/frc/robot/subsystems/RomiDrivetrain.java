// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;

public class RomiDrivetrain extends SubsystemBase {
	private static final double kCountsPerRevolution = 1440.0;
	private static final double kWheelDiameterMeter = 0.07; // 70 mm

	// The Romi has the left and right motors set to
	// PWM channels 0 and 1 respectively
	private final Spark m_leftMotor = new Spark(0);
	private final Spark m_rightMotor = new Spark(1);

	// The Romi has onboard encoders that are hardcoded
	// to use DIO pins 4/5 and 6/7 for the left and right
	private final Encoder m_leftEncoder = new Encoder(4, 5);
	private final Encoder m_rightEncoder = new Encoder(6, 7);

	private final RomiGyro m_gyro = new RomiGyro();

	@SuppressWarnings("unused")
	private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer(Range.k2G);

	// Set up the differential drive controller
	private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

	private final PIDController m_leftPID = new PIDController(Constants.kPDriveVel, 0, 0);
	private final PIDController m_rightPID = new PIDController(Constants.kPDriveVel, 0, 0);

	private final DifferentialDrivePoseEstimator m_globalPoseObserver = new DifferentialDrivePoseEstimator(
		m_gyro.getRotation2d(),
		Constants.kTeleopStartPose,
		VecBuilder.fill(0.02, 0.02, 0.01, 0.02, 0.02),
		VecBuilder.fill(0.02, 0.02, 0.01),
		VecBuilder.fill(0.1, 0.1, 0.01),
		0.02
	);

	private final Field2d m_field = new Field2d();

	/** Creates a new RomiDrivetrain. */
	public RomiDrivetrain() {
		// Use meters as unit for encoder distances
		m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
		m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
		resetEncoders();

		SmartDashboard.putData("field", m_field);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		m_field.setRobotPose(m_globalPoseObserver.update(m_gyro.getRotation2d(), getWheelSpeeds(),
			m_leftEncoder.getDistance(), m_rightEncoder.getDistance()));
		SmartDashboard.putNumber("Left Setpoint", m_leftPID.getSetpoint());
		SmartDashboard.putNumber("Right Setpoint", m_rightPID.getSetpoint());
		SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getRate());
		SmartDashboard.putNumber("Right Velocity", m_rightEncoder.getRate());
		SmartDashboard.putNumber("Left Error", m_leftPID.getPositionError());
		SmartDashboard.putNumber("Right Error", m_rightPID.getPositionError());
	}

	public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		m_diffDrive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
	}

	public void resetEncoders() {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
	}

	public void stopDrivetrain() {
		m_diffDrive.stopMotor();
	}

	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		m_globalPoseObserver.resetPosition(pose, m_gyro.getRotation2d());
	}

	public Pose2d getPose() {
		return m_globalPoseObserver.getEstimatedPosition();
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		m_leftMotor.setVoltage(leftVolts * Constants.kLeftMotorBias);
		m_rightMotor.setVoltage(-rightVolts / Constants.kLeftMotorBias);
		m_diffDrive.feed();
	}

	public PIDController getLeftPID() {
		return m_leftPID;
	}

	public PIDController getRightPID() {
		return m_rightPID;
	}

}
