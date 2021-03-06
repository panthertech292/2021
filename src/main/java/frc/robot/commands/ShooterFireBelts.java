// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.BeltConstants;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterFireBelts extends ParallelDeadlineGroup {
  /** Creates a new ShooterFireBelts. */
  private final ShooterSubsystem ShooterSubsystem;
  private final BeltSubsystem BeltSubsystem;
  public ShooterFireBelts(ShooterSubsystem s_ShooterSubsystem, BeltSubsystem s_BeltSubsystem) {
    super(new BeltSchedule(s_BeltSubsystem));
    ShooterSubsystem = s_ShooterSubsystem;
    BeltSubsystem = s_BeltSubsystem;
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    
     addCommands(
       new ShooterFireFull(s_ShooterSubsystem)
     );
  }
}
