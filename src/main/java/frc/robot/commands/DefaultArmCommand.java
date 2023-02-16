package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Command that tilts the wrist up if not doing anything else
 */
public class DefaultArmCommand extends CommandBase {

  private final ArmSubsystem armSubsystem;

  public DefaultArmCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void execute() {
    armSubsystem.moveToPosition(0);
  }
  
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }
}