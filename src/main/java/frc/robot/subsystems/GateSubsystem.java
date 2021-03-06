
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
  //Servos
  private final Servo Gate1Servo;
  private final Servo Gate2Servo;
  //Sensors
  public final DigitalInput Ball1Sensor;
  //Variables
  private static double v_servo1Value;
  private static double v_servo2Value;
  private int v_shootState;
  private final Timer timer;

  public GateSubsystem() {
    //Servos
    Gate1Servo = new Servo(GateConstants.Gate1Servo);
    Gate2Servo = new Servo(GateConstants.Gate2Servo);
    addChild("Gate1Servo", Gate1Servo);
    //Variables
    v_shootState = 0;
    v_servo1Value = GateConstants.gate1Max;
    v_servo2Value = GateConstants.gate2Max;
    //Sensors
    Ball1Sensor = new DigitalInput(GateConstants.Ball1Sensor);
    timer = new Timer();
  }
  //Gate 1
  public void Gate1(){
    Gate1Servo.set(v_servo1Value);
  }
  public void Gate1Mid() {
    v_servo1Value = GateConstants.gate1Mid;
  }
  public void Gate1Down() {
    v_servo1Value = GateConstants.gate1Down;
  }
  public void Gate1Max() {
    v_servo1Value = GateConstants.gate1Max;
  }
  //Gate 2
  public void Gate2(){
    Gate2Servo.set(v_servo2Value);
  }
  public void Gate2Mid() {
    v_servo2Value = GateConstants.gate2Mid;
  }
  public void Gate2Down() {
    v_servo2Value = GateConstants.gate2Down;
  }
  public void Gate2Max() {
    v_servo2Value = GateConstants.gate2Max;
  }
  public boolean isBallPresent() {  
    return !Ball1Sensor.get();
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
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Gate1();
    Gate2();
  }
}
