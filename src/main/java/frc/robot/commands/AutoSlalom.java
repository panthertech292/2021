// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSlalom extends SequentialCommandGroup {
  /** Creates a new AutoSlalom. */
  private final DriveSubsystem DriveSubsystem;
  public AutoSlalom(DriveSubsystem s_DriveSubsystem) {
    DriveSubsystem = s_DriveSubsystem;

   addRequirements(s_DriveSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoForward(s_DriveSubsystem, .5, 0.775, .85),
      new AutoForward(s_DriveSubsystem, .775, 0.4, 1.1),
    new AutoForwardPID(DriveSubsystem, .7, .65, 16),
    new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
    new AutoForward(s_DriveSubsystem, 0.2, 0.65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 90, 2.15789),
    new AutoForward(s_DriveSubsystem, 0.65, 0.2, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .65, .2, 90, 2.15789),
    new AutoForwardPID(DriveSubsystem, .7, .65, 120),
    new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
    new AutoForward(s_DriveSubsystem, 0.2, 0.65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .65, .2, 90, 2.15789),
    new AutoForward(s_DriveSubsystem, 0.2, 0.65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 360, 2.15789),
    new AutoForward(s_DriveSubsystem, 0.2, 0.65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .65, .2, 90, 2.15789),
    new AutoForwardPID(DriveSubsystem, .7, .65, 120),
    new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
    new AutoForward(s_DriveSubsystem, 0.65, 0.2, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .65, .2, 90, 2.15789),
    new AutoForward(s_DriveSubsystem, 0.2, 0.65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 90, 2.15789),
    new AutoForwardPID(DriveSubsystem, .7, .65, 30),
    new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
    new DriveUntilBounceSensor(s_DriveSubsystem, .3, .3)

    );
  }
}
