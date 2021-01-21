
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GateConstants;

public class GateSubsystem extends SubsystemBase {
  /**
   * Creates a new GateSubsystem.
   */
  private final Servo gate1Servo;
  private final Servo gate2Servo;
  private int m_shootState;
  
  public final DigitalInput ball1Sensor;
  private static double servo1value;
  private static double servo2value;
  
  private final Timer timer;
  public GateSubsystem() {
    gate1Servo = new Servo(GateConstants.gate1Servo);
    gate2Servo = new Servo(GateConstants.gate2Servo);
    m_shootState = 0;
    servo1value = GateConstants.gate1Max;
    servo2value = GateConstants.gate2Max;

    ball1Sensor = new DigitalInput(GateConstants.ball1sensor);
    timer = new Timer();

    addChild("Gate1Servo", gate1Servo);

  }
  /*public void gateControl() {
    gate1Servo.set(servo1value);
  }*/

  public void Gate1(){
    gate1Servo.set(servo1value);
  }

  public void Gate1Mid() {
    servo1value = GateConstants.gate1Mid;
  }
  public void Gate1Down() {
    servo1value = GateConstants.gate1Down;
  }
  public void Gate1Max() {
    servo1value = GateConstants.gate1Max;
  }
  //Gate 2
  public void Gate2(){
    gate2Servo.set(servo2value);
  }
  public void Gate2Mid() {
    servo2value = GateConstants.gate2Mid;
  }
 
  public void Gate2Down() {
    servo2value = GateConstants.gate2Down;
  }
  public void Gate2Max() {
    servo2value = GateConstants.gate2Max;
  }
  
  public boolean isBallPresent() {  
    //System.out.println(ball1Sensor.get());
    //return ball1Sensor.get();
    return !ball1Sensor.get();
  
    }
  public boolean isShooterMotorSpunUp(){
    return  getTimerValue()>GateConstants.ShooterSpinUpTime;
  }
  
  public boolean isShooterDone() {
    return  getTimerValue()>GateConstants.gateUpTime;
  }

  public boolean isLoaderDone() {
    return  getTimerValue()>GateConstants.gateDownTime;
  }

    public void resetTimer() {
      timer.reset();
      timer.start();
    }
  
  public double getTimerValue() {
    return Math.abs(timer.get());
  }

public void resetGates() {
  Gate1Down();
  resetTimer();

}

 public void ShootOne() {
  Gate1Mid();
  resetTimer();

 }

 public void LoadNext() {
  Gate1Down();
  resetTimer();
 }

  
   
   
  public void resetShootStateEnd(){
    Gate1Down();
  }
  //
  //
  public void resetShootStateContinue(){
    m_shootState = 1;
  }
  //
  //
  public int getShootState(){
    return m_shootState;
  }
//
//
  

 /* public boolean isFireGatedButtonStillPressed() {
    return RobotContainer.isFireGatedPressed();
  }

  public boolean isFireGatedButtonSmartStillPressed() {
    return RobotContainer.isFireGatedSmartPressed();
  }
*/
  @Override
  public void periodic() {
    Gate1();
    Gate2();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("GateShootState", m_shootState);
    SmartDashboard.putNumber("GateTimer", getTimerValue());
    SmartDashboard.putBoolean("Ball Sensor", isBallPresent());
  }
}
