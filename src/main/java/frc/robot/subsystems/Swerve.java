// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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

  public static class SwerveModule {
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double ANGLE_GEAR_RATIO = 150.0 / 7.0;
    public static final double DRIVE_GEAR_RATIO =
        1.0 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double ROTATION_TO_METER_RATIO = DRIVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE;

    public static final double ANGLE_KP = 5; // TODO actually tune these, this is just a guess rn lol
    public static final double DRIVE_KS = 0.1;
    public static final double DRIVE_KV = 2.4;
    public static final double DRIVE_KP = 0.6;

    public final int id;
    public final String name;

    private final CANcoder angleEncoder;
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final VelocityVoltage driveMotorControl;
    private final PositionVoltage angleMotorControl;

    public SwerveModule(
        int id,
        String name,
        int driveMotorID,
        int angleMotorID,
        int angleEncoderID,
        double cancoderOffset) {
      this.id = id;
      this.name = name;

      angleEncoder = new CANcoder(angleEncoderID);
      driveMotor = new TalonFX(driveMotorID);
      angleMotor = new TalonFX(angleMotorID);

      driveMotorControl = new VelocityVoltage(0);
      angleMotorControl = new PositionVoltage(0);

      var angleEncoderConfig = new CANcoderConfiguration();
      angleEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
      angleEncoderConfig.MagnetSensor.MagnetOffset = cancoderOffset;
      angleEncoderConfig.MagnetSensor.AbsoluteSensorRange =
          AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
      angleEncoder.getConfigurator().apply(angleEncoderConfig);

      var driveMotorConfig = new TalonFXConfiguration();
      driveMotorConfig.Slot0.kS = DRIVE_KS;
      driveMotorConfig.Slot0.kV = DRIVE_KV;
      driveMotorConfig.Slot0.kP = DRIVE_KP;
      driveMotorConfig.Feedback.SensorToMechanismRatio = ROTATION_TO_METER_RATIO;
      driveMotor.getConfigurator().apply(driveMotorConfig);

      var angleMotorConfig = new TalonFXConfiguration();
      angleMotorConfig.Slot0.kP = ANGLE_KP;
      angleMotorConfig.Feedback.SensorToMechanismRatio = ANGLE_GEAR_RATIO;
      angleMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
      angleMotor.getConfigurator().apply(angleMotorConfig);
    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(getVelocity(), getAngle());
    }

    public Rotation2d getAngle() {
      return Rotation2d.fromRotations(angleMotor.getPosition().getValue());
    }

    public double getVelocity() {
      return driveMotor.getVelocity().getValue();
    }

    public double getDrivePosition() {
      return driveMotor.getPosition().getValue();
    }
    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(getDrivePosition(), getAngle());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
      // Optimize the reference state to avoid spinning further than 90 degrees
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, getAngle());
      angleMotor.setControl(angleMotorControl.withPosition(state.angle.getRotations()));
      driveMotor.setControl(driveMotorControl.withVelocity(state.speedMetersPerSecond));
    }

    public void resetToAbsolute() {
      double encoderAngle = angleEncoder.getPosition().waitForUpdate(0.1).getValue();
      angleMotor.setRotorPosition(encoderAngle * ANGLE_GEAR_RATIO);
    }
  }
}
