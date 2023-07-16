package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  // The robot's subsystems
  public final Swerve swerve = new Swerve();
  public final Controls controls = new Controls(this);

  public RobotContainer() {
    controls.configure();
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
