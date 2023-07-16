// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class Swerve extends SubsystemBase {
  public static final int GYRO_ID = 1;
  public static final double MAX_SPEED = 5;
  public static final double MAX_ANGULAR_VELOCITY = 5;

  public static final double TRACK_WIDTH = Units.inchesToMeters(23.75);
  public static final double WHEEL_BASE = TRACK_WIDTH;

  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

  // Robot swerve modules
  private final List<SwerveModule> modules =
      List.of(
          new SwerveModule(0, "FL", 2, 1, 1, 0.1802),
          new SwerveModule(1, "FR", 4, 3, 2, -0.3169),
          new SwerveModule(2, "BL", 8, 7, 4, -0.1667),
          new SwerveModule(3, "BR", 6, 5, 3, 0.4631));

  private final Pigeon2 gyro = new Pigeon2(GYRO_ID);

  // Odometry class for tracking robot pose
  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(DRIVE_KINEMATICS, gyro.getRotation2d(), getModulePositions());

  /** Creates a new DriveSubsystem. */
  public Swerve() {
    resetToAbsolute();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(gyro.getRotation2d(), getModulePositions());
  }

  public SwerveModulePosition[] getModulePositions() {
    return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  // TODO: replace with 2024 WPILib implementation
  public static ChassisSpeeds correctSpeeds(ChassisSpeeds speeds) {
    final double dtSeconds = TimedRobot.kDefaultPeriod;
    var desiredDeltaPose =
        new Pose2d(
            speeds.vxMetersPerSecond * dtSeconds,
            speeds.vyMetersPerSecond * dtSeconds,
            new Rotation2d(speeds.omegaRadiansPerSecond * dtSeconds));
    var twist = new Pose2d().log(desiredDeltaPose);
    return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void joystickDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= MAX_SPEED;
    ySpeed *= MAX_SPEED;
    rot *= MAX_ANGULAR_VELOCITY;
    var wantedSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose().getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    wantedSpeeds = correctSpeeds(wantedSpeeds);
    var desiredStates = DRIVE_KINEMATICS.toSwerveModuleStates(wantedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, wantedSpeeds, MAX_SPEED, MAX_SPEED, MAX_ANGULAR_VELOCITY);
    setModuleStates(desiredStates);
  }

  /**
   * Method to drive the robot using chassis speeds. Maybe useful for PathPlanner?
   *
   * @param wantedSpeeds The desired robot speeds.
   */
  public void setChassisSpeeds(ChassisSpeeds wantedSpeeds) {
    wantedSpeeds = correctSpeeds(wantedSpeeds);
    var desiredStates = DRIVE_KINEMATICS.toSwerveModuleStates(wantedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);
    setModuleStates(desiredStates);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    modules.forEach(m -> m.setDesiredState(desiredStates[m.id]));
  }

  /** Resets the angle motors to the absolute encoder positions. */
  public void resetToAbsolute() {
    modules.forEach(SwerveModule::resetToAbsolute);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.setYaw(0);
  }
}
