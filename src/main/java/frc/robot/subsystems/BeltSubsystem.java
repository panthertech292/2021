// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeltConstants;

public class BeltSubsystem extends SubsystemBase {
  /** Creates a new BeltSubsystem. */
  private final WPI_TalonSRX FrontBeltMotor;
  private final WPI_TalonSRX BackBeltMotor;
  private final WPI_TalonSRX BottomBeltMotor;

  private int v_printCount;

  public BeltSubsystem() {
    FrontBeltMotor = new WPI_TalonSRX(BeltConstants.kFrontBeltMotor);
    BackBeltMotor = new WPI_TalonSRX(BeltConstants.kBackBeltMotor);
    BottomBeltMotor = new WPI_TalonSRX(BeltConstants.kBottomBeltMotor);

    v_printCount = 0;
  }





  public void timedPrintOut(){
    if(v_printCount % 100 == 0){
      //Enter print statements here!
      
    }
    v_printCount = v_printCount + 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}