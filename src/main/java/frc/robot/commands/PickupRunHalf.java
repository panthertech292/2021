/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PickupSubsystem;

public class PickupRunHalf extends CommandBase {
  private final PickupSubsystem PickupSubsystem;

  /**
   * Creates a new PickupRunHalf.
   */
  public PickupRunHalf(PickupSubsystem s_PickupSubsystem){
    PickupSubsystem = s_PickupSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_PickupSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PickupSubsystem.ChangeSetpoint(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PickupSubsystem.ChangeSetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
