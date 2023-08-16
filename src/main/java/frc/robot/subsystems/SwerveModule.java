package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  public static final double WHEEL_DIAMETER = // Units.inchesToMeters(4)
      4 * 0.0254;
  public static final double ANGLE_GEAR_RATIO = 150.0 / 7.0;
  public static final double DRIVE_GEAR_RATIO =
      1.0 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
  public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
  public static final double ROTATION_TO_METER_RATIO = DRIVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE;

  public static final double ANGLE_KP = 10; // TODO actually tune these, this is just a guess rn lol
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
    driveMotorConfig.Audio.BeepOnBoot = true;
    // TODO figure out what the optimal current limits are
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 60;
    driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;
    // TODO brake?
    driveMotor.getConfigurator().apply(driveMotorConfig);

    var angleMotorConfig = new TalonFXConfiguration();
    angleMotorConfig.Slot0.kP = ANGLE_KP;
    angleMotorConfig.Feedback.SensorToMechanismRatio = ANGLE_GEAR_RATIO;
    angleMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    angleMotorConfig.Audio.BeepOnBoot = true;
    // TODO figure out what the optimal current limits are
    angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    angleMotorConfig.CurrentLimits.SupplyCurrentLimit = 25;
    angleMotorConfig.CurrentLimits.SupplyTimeThreshold = 40;
    angleMotorConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;
    angleMotorConfig.MotorOutput.Inverted =
        InvertedValue.Clockwise_Positive; // todo make this a constant or something
    // TODO brake?
    angleMotor.getConfigurator().apply(angleMotorConfig);

    driveSimState = driveMotor.getSimState();
    angleSimState = angleMotor.getSimState();
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

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getAngle());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getAngle());

    SmartDashboard.putNumber("module desired angle " + name, state.angle.getRotations());
    SmartDashboard.putNumber("module desired velocity " + name, state.speedMetersPerSecond);

    SmartDashboard.putNumber("module actual angle " + name, getAngle().getRotations());
    SmartDashboard.putNumber("module actual velocity " + name, getVelocity());

    angleMotor.setControl(angleMotorControl.withPosition(state.angle.getRotations()));
    driveMotor.setControl(driveMotorControl.withVelocity(state.speedMetersPerSecond));
  }

  public void resetToAbsolute() {
    double encoderAngle = angleEncoder.getPosition().waitForUpdate(0.1).getValue();
    angleMotor.setRotorPosition(encoderAngle * ANGLE_GEAR_RATIO);
  }

  // Simulation stuff starts here

  private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), DRIVE_GEAR_RATIO, 0.025);
  private FlywheelSim angleSim = new FlywheelSim(DCMotor.getFalcon500(1), ANGLE_GEAR_RATIO, 0.004);
  private TalonFXSimState driveSimState;
  private TalonFXSimState angleSimState;

  public void simulationPeriodic() {
    driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    angleSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    driveSim.setInputVoltage(driveSimState.getMotorVoltage());
    angleSim.setInputVoltage(angleSimState.getMotorVoltage());

    SmartDashboard.putNumber(
        "drive velocity input voltage " + name, driveSimState.getMotorVoltage());
    SmartDashboard.putNumber(
        "angle velocity input voltage " + name, angleSimState.getMotorVoltage());

    driveSim.update(0.02);
    angleSim.update(0.02);

    double driveVelocity =
        Units.radiansToRotations(driveSim.getAngularVelocityRadPerSec() * DRIVE_GEAR_RATIO);
    double angleVelocity =
        Units.radiansToRotations(angleSim.getAngularVelocityRadPerSec() * ANGLE_GEAR_RATIO);

    SmartDashboard.putNumber("drive velocity motor " + name, driveVelocity);
    SmartDashboard.putNumber("angle velocity motor " + name, angleVelocity);

    driveSimState.addRotorPosition(driveVelocity * 0.02);
    angleSimState.addRotorPosition(angleVelocity * 0.02);
    driveSimState.setRotorVelocity(driveVelocity);
    angleSimState.setRotorVelocity(angleVelocity);
  }

  public double getLoad() {
    return driveSim.getCurrentDrawAmps() + angleSim.getCurrentDrawAmps();
  }
}
