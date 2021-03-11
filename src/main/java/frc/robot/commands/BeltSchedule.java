// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BeltConstants;
import frc.robot.subsystems.BeltSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BeltSchedule extends SequentialCommandGroup {
  /** Creates a new BeltSchedule. */
  private final BeltSubsystem BeltSubsystem;
  
  public BeltSchedule(BeltSubsystem s_BeltSubsystem) {
    BeltSubsystem = s_BeltSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BeltWait(s_BeltSubsystem, 5.0),
      new BeltMoveTimed(s_BeltSubsystem)
      /* new BeltMoveTimed(s_BeltSubsystem, 0.0, 0.5),
       new BeltMoveTimed(s_BeltSubsystem, BeltConstants.kBeltForwardSpeed, 0.3),
       new BeltMoveTimed(s_BeltSubsystem, 0.0, 0.5),
       new BeltMoveTimed(s_BeltSubsystem, BeltConstants.kBeltForwardSpeed, 0.3)
    */);
  }
}
