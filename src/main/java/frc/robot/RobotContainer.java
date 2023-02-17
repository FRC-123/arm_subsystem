// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;
import static frc.robot.Constants.TeleopDriveConstants.DEADBAND;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultHiArmCommand;
import frc.robot.subsystems.HiArmSubsystem;
import frc.robot.commands.DefaultLoArmCommand;
import frc.robot.subsystems.LoArmSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CommandXboxController controller = new CommandXboxController(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final HiArmSubsystem armSubsystem = new HiArmSubsystem();
  private final DefaultHiArmCommand defaultArmCommand = new DefaultHiArmCommand(armSubsystem);

  private final LoArmSubsystem LoarmSubsystem = new LoArmSubsystem();
  private final DefaultLoArmCommand defaultLoArmCommand = new DefaultLoArmCommand(LoarmSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    armSubsystem.setDefaultCommand(defaultArmCommand);
    LoarmSubsystem.setDefaultCommand(defaultLoArmCommand);
  }

 /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Arm
    controller.x().whileTrue(
      startEnd(() -> armSubsystem.moveArm(0.3), armSubsystem::stop, armSubsystem));
    controller.a().whileTrue(
      startEnd(() -> armSubsystem.moveArm( -0.3), armSubsystem::stop, armSubsystem));

    controller.y().whileTrue(
      startEnd(() -> LoarmSubsystem.moveArm(0.3), LoarmSubsystem::stop, LoarmSubsystem));
    controller.b().whileTrue(
      startEnd(() -> LoarmSubsystem.moveArm( -0.3), LoarmSubsystem::stop, LoarmSubsystem));
      
    controller.start().whileTrue(
      startEnd(() -> armSubsystem.moveToPosition(90), armSubsystem::stop, armSubsystem));
    controller.back().whileTrue(
      startEnd(() -> armSubsystem.moveToPosition(-45), armSubsystem::stop, armSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_exampleSubsystem);
  //   // return(0);
  // }
}
