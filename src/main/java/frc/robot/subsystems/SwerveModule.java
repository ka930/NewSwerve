package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
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
    driveMotor.setControl(new VelocityVoltage(state.speedMetersPerSecond));
    angleMotor.setControl(new PositionVoltage(state.angle.getRotations()));
  }

  public void resetToAbsolute() {
    double encoderAngle = angleEncoder.getPosition().waitForUpdate(0.1).getValue();
    angleMotor.setRotorPosition(encoderAngle * ANGLE_GEAR_RATIO);
  }
}
