package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public class Controls {
  private static final int DRIVER_PORT = 0;

  public static final double STICK_DEADBAND = 0.06;
  public static final double STICK_LINEAR_COEFFICIENT = 1;
  public static final double STICK_CUBIC_COEFFICIENT = 1 - STICK_LINEAR_COEFFICIENT;

  private final Swerve swerve;

  private CommandXboxController driver = new CommandXboxController(DRIVER_PORT);

  private double adjustStickValue(double value) {
    return STICK_LINEAR_COEFFICIENT * value + STICK_CUBIC_COEFFICIENT * Math.pow(value, 3);
  }
  // todo: pull in GenericGamepad from 2023 robot code and add sim gamepad (F310) support
  private DoubleSupplier leftY =
      Robot.isSimulation() ? () -> -driver.getRawAxis(1) : () -> -driver.getLeftY();
  private DoubleSupplier leftX =
      Robot.isSimulation() ? () -> -driver.getRawAxis(0) : driver::getLeftX;
  private DoubleSupplier rightX =
      Robot.isSimulation() ? () -> -driver.getRawAxis(2) : driver::getRightX;

  public void configure() {
    swerve.setDefaultCommand(
        swerve.run(
            () -> {
              double xSpeed =
                  adjustStickValue(MathUtil.applyDeadband(leftY.getAsDouble(), STICK_DEADBAND));
              double ySpeed =
                  adjustStickValue(MathUtil.applyDeadband(leftX.getAsDouble(), STICK_DEADBAND));
              double rSpeed =
                  adjustStickValue(MathUtil.applyDeadband(rightX.getAsDouble(), STICK_DEADBAND));
              swerve.joystickDrive(xSpeed, ySpeed, rSpeed, true);
            }));

    driver.y().onTrue(Commands.runOnce(swerve::zeroHeading));
  }

  public Controls(RobotContainer bot) {
    this.swerve = bot.swerve;
  }
}
