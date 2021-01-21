/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants.PickupConstants;
import edu.wpi.first.wpilibj.Timer;

public class PickupSubsystem extends SubsystemBase {
  /**
   * Creates a new PickupSubsystem.
   */
  private final WPI_TalonSRX pickupMotorTop;
  private final WPI_TalonSRX pickupMotorBottom;
  private double v_pickupSpeedTop;
  private double v_pickupSpeedBottom;
  private final Timer Timer;
  public PickupSubsystem() {
    pickupMotorTop = new WPI_TalonSRX(PickupConstants.kPickupMotorTop);
    pickupMotorBottom = new WPI_TalonSRX(PickupConstants.kPickupMotorBottom);

    pickupMotorTop.setNeutralMode(NeutralMode.Brake);
    pickupMotorBottom.setNeutralMode(NeutralMode.Brake);

    Timer = new Timer();
  }
  public void resetTimer() {
    Timer.reset();
    Timer.start();
  }

  public Double getTimerValue() {
    return Math.abs(Timer.get());
  }
  public void ChangeSetpoint(double pickupspeed) {
    v_pickupSpeedTop = pickupspeed; //* m_ratio;
    v_pickupSpeedBottom = pickupspeed;
    
  }
  public void ChangeSetpointsRev(double pickupspeed) {
    v_pickupSpeedTop = -pickupspeed; // * m_ratio;
    v_pickupSpeedBottom = -pickupspeed;

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Jeb!
    pickupMotorTop.set(v_pickupSpeedTop);
    pickupMotorBottom.set(v_pickupSpeedBottom);
    
  }
}
