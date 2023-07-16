package frc.robot.subsystems;

import static frc.robot.subsystems.Swerve.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private final CANcoder angleEncoder;
  private final TalonFX driveMotor;
  private final TalonFX angleMotor;
  public final int id;
  public final String name;

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

    var angleEncoderConfig = new CANcoderConfiguration();
    angleEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    angleEncoderConfig.MagnetSensor.MagnetOffset = cancoderOffset;
    angleEncoder.getConfigurator().apply(angleEncoderConfig);

    var driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.Feedback.SensorToMechanismRatio = ROTATION_TO_METER_RATIO;
    driveMotor.getConfigurator().apply(driveMotorConfig);

    var angleMotorConfig = new TalonFXConfiguration();
    angleMotorConfig.Feedback.SensorToMechanismRatio = ANGLE_GEAR_RATIO;
    angleMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    angleMotor.getConfigurator().apply(angleMotorConfig);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getCurrentVelocity(), getCurrentAngle());
  }

  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromRotations(angleMotor.getPosition().getValue());
  }

  public double getCurrentVelocity() {
    return driveMotor.getVelocity().getValue();
  }

  public double getCurrentPosition() {
    return driveMotor.getPosition().getValue();
  }
  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getCurrentPosition(), getCurrentAngle());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getCurrentAngle());

    // Calculate the turning motor output from the turning PID controller.
    driveMotor.set(state.speedMetersPerSecond);
    angleMotor.set(state.angle.getRotations());
  }

  public void resetToAbsolute() {
    double encoderAngle = angleEncoder.getPosition().waitForUpdate(0.1).getValue();
    angleMotor.setRotorPosition(encoderAngle * ANGLE_GEAR_RATIO);
  }
}
