package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Swerve;

public class Controls {
  private static final int DRIVER_PORT = 0;

  public static final double STICK_DEADBAND = 0.06;
  public static final double STICK_LINEAR_COEFFICIENT = 0.5;
  public static final double STICK_CUBIC_COEFFICIENT = 1 - STICK_LINEAR_COEFFICIENT;

  private final Swerve swerve;

  private CommandPS4Controller driver = new CommandPS4Controller(DRIVER_PORT);

  private double adjustStickValue(double value) {
    return STICK_LINEAR_COEFFICIENT * value + STICK_CUBIC_COEFFICIENT * Math.pow(value, 3);
  }

  public void configure() {
    swerve.setDefaultCommand(
        swerve.run(
            () -> {
              double xSpeed =
                  adjustStickValue(MathUtil.applyDeadband(-driver.getLeftY(), STICK_DEADBAND));
              double ySpeed =
                  adjustStickValue(MathUtil.applyDeadband(driver.getLeftX(), STICK_DEADBAND));
              double rSpeed =
                  adjustStickValue(MathUtil.applyDeadband(driver.getRightY(), STICK_DEADBAND));
              swerve.joystickDrive(xSpeed, ySpeed, rSpeed, true);
            }));

    driver.triangle().onTrue(Commands.runOnce(swerve::zeroHeading));
  }

  public Controls(RobotContainer bot) {
    this.swerve = bot.swerve;
  }
}
