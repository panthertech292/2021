package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final WPI_TalonSRX FrontLeftMotor;
  private final WPI_TalonSRX FrontRightMotor;
  private final WPI_TalonSRX BackLeftMotor;
  private final WPI_TalonSRX BackRightMotor;

  private final SpeedControllerGroup LeftSide;
  private final SpeedControllerGroup RightSide;
 
private final Timer Timer;

    private final DifferentialDrive Drive;

    private double v_leftSpeed;
    private double v_rightSpeed;
    private double v_setPointLeft;
    private double v_setPointRight;

    private double v_zeroLeftPosition;
    private double v_zeroRightPosition;
    private double v_zeroAngle;
    //private double m_leftDistance;
    private double v_deltaAngle;
    private double c_encoderConversion;
     //Put this into constants.java
    private int c_modeTeleop = 0;
    private int c_modeSetPoint = 1;
    private int v_driveMode = c_modeTeleop;
    private DigitalInput v_testSwitch;
    private DigitalInput opticalSensor;
    private DigitalInput testDio;
    private AnalogInput sonarSensor;
    private double v_limeLightX;
    private double v_limeLightY;
    private double v_limeLightArea;
    private double c_VisionAreaTarget;
    private int v_loopCount;
    private int v_visionOverride;

    private NetworkTableEntry v_TableEntrytx;
    private NetworkTableEntry v_TableEntryty;
    private NetworkTableEntry v_TableEntryta;

    private NetworkTable table;

    AHRS ahrs;
 



  public DriveSubsystem() {

    FrontLeftMotor = new WPI_TalonSRX(DriveConstants.kFrontLeftMotor);
    FrontRightMotor = new WPI_TalonSRX(DriveConstants.kFrontRightMotor);
    BackLeftMotor = new WPI_TalonSRX(DriveConstants.kBackLeftMotor);
    BackRightMotor = new WPI_TalonSRX(DriveConstants.kBackRightMotor);
    v_testSwitch = new DigitalInput(DriveConstants.kTestSwitch);
    opticalSensor = new DigitalInput(DriveConstants.kOpticalPort);
    sonarSensor = new AnalogInput(DriveConstants.kSonarPort);
    testDio = new DigitalInput(6);
  
    LeftSide = new SpeedControllerGroup(FrontLeftMotor,BackLeftMotor);
    RightSide = new SpeedControllerGroup(FrontRightMotor, BackRightMotor);
    
    table = NetworkTableInstance.getDefault().getTable("limelight");
    v_TableEntrytx = table.getEntry("tx");
    v_TableEntryty = table.getEntry("ty");
    v_TableEntryta = table.getEntry("ta");
    
    v_limeLightX = v_TableEntrytx.getDouble(0.0);
    v_limeLightY = v_TableEntryty.getDouble(0.0);
    v_limeLightArea = v_TableEntryta.getDouble(0.0);

    c_VisionAreaTarget = DriveConstants.kVisionAreaTarget;
    v_loopCount = 0;
    v_visionOverride = 0;

    Drive = new DifferentialDrive(LeftSide,RightSide);

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP); 
  } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
  }

  

    

    BackLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    BackRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    v_setPointLeft = 0.0;
    v_setPointRight = 0.0;

    
    Timer = new Timer();
    c_encoderConversion = 4096;


    addChild("Drive", Drive);
    addChild("Left Side", LeftSide);
    addChild("Right Side", RightSide);

    FrontLeftMotor.setNeutralMode(NeutralMode.Brake);
    FrontRightMotor.setNeutralMode(NeutralMode.Brake);
    BackRightMotor.setNeutralMode(NeutralMode.Brake);
    BackLeftMotor.setNeutralMode(NeutralMode.Brake);
      
  }
  public void updateLimeLight() {
    
    v_limeLightX = v_TableEntrytx.getDouble(0.0);
    v_limeLightY = v_TableEntryty.getDouble(0.0);
    v_limeLightArea = v_TableEntryta.getDouble(0.0);
  }


  public void differentialDrive(double leftspeed, double rightspeed) {
    v_leftSpeed = -leftspeed;
    v_rightSpeed = - rightspeed;
    Drive.tankDrive(v_leftSpeed, v_rightSpeed);
  }
  public void resetTimer() {
    Timer.reset();
    Timer.start();
  }
  
  public double getTimerValue() {
    return Math.abs(Timer.get());
  }
  
  //Encoder subroutines 
  public double getLeftEncoderValue() {
      return BackLeftMotor.getSelectedSensorPosition();
  }
  
  public double getLeftPosition(){
    return ((BackLeftMotor.getSelectedSensorPosition() - v_zeroLeftPosition)/c_encoderConversion)*(Math.PI*6);
  }
  
  public void zeroLeftPosition() {
    v_zeroLeftPosition = BackLeftMotor.getSelectedSensorPosition();
  }
  public double getZeroPosition(){
    return v_zeroLeftPosition;
  }
  public void zeroAngle() {
    v_zeroAngle = ahrs.getYaw();
  }
  public double getZeroAngle(){
    return v_zeroAngle;
  }
  public double getCurrentAngle(){
    return ahrs.getYaw();
  }
  public void zeroRightPosition(){
    v_zeroRightPosition = BackRightMotor.getSelectedSensorPosition();
  }
  public boolean gyroFinish( double angle){
    v_deltaAngle = (getCurrentAngle() - getZeroAngle());
    if (angle <= v_deltaAngle){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean totalFinish(double angle){
   return (gyroFinish(angle) && encoderFinish((angle/360)*2*Math.PI*21));
  }

  public void deltaTurn(){
   double v_deltaAngle = (getCurrentAngle() - getZeroAngle());
   double gyroDistance = (v_deltaAngle/360) *2*Math.PI*21;
   double encoderDistance = (Math.abs(getLeftPosition()) + Math.abs(getRightPosition())) / 2;
   double encoderAngle = ((encoderDistance*360)/(2*Math.PI*10.5));
  // System.out.println("Gyro Distance" +  gyroDistance);
  // System.out.println("Encoder Distance" + encoderDistance);
  // System.out.println("Delta Turn Distance" + (gyroDistance - encoderDistance));
  /*System.out.println("Gyro Angle" +  v_deltaAngle);
  System.out.println("Encoder Angle" +  encoderAngle);
  System.out.println("Delta Turn Angle" + (v_deltaAngle - encoderAngle));
*/

   //return (gyroDistance - encoderDistance);

  }

  public void zeroDistanceSensors(){
    zeroLeftPosition();
    zeroAngle();
    zeroRightPosition();
  }
  //Vision
  public boolean visionFinish(){
   
    if (- 1 <= v_limeLightX && v_limeLightX <= 1 || v_visionOverride == 1 ){
      v_visionOverride = 0;
      return true;}
    
    else{
      return false;
    }
  }
  public boolean visionFinishDistance(){
    if (c_VisionAreaTarget - 0.1 <= v_limeLightArea && c_VisionAreaTarget + 0.1 >= v_limeLightArea){
      return true;
    }
    else{
      return false;
    }
  }
  //Actually turns robot right, aimed too far left
  public void visionAlignLeft(){
    
    if (v_limeLightX > 1){
      v_loopCount = v_loopCount+1;
      System.out.println(v_loopCount);
      System.out.println("Trying to align left!");
      //if (RobotContainer.getRobotID() == Constants.kProductionBotID){
      //  changePowerSetPoints(DriveConstants.kProdBotVisionAlignSpeedDefault, -DriveConstants.kProdBotVisionAlignSpeedDefault);
      //}
      //else{
      //  changePowerSetPoints(DriveConstants.kVisionAlignSpeedDefault, -DriveConstants.kVisionAlignSpeedDefault);
      //}
      changePowerSetPoints(RobotContainer.setMotorSpeed(DriveConstants.kProdBotVisionAlignSpeedDefault, DriveConstants.kVisionAlignSpeedDefault), -RobotContainer.setMotorSpeed(DriveConstants.kProdBotVisionAlignSpeedDefault, DriveConstants.kVisionAlignSpeedDefault));
      //50 loops = 1 sec?
      if(v_loopCount>=150){
        System.out.println("Alignment Aborted");
        v_loopCount = 0;
        v_visionOverride = 1;
      }
    }

 
  }
  //Actually turns robot left, aimed too far right
  public void visionAlignRight(){
    
    if (v_limeLightX < -1){
      v_loopCount = v_loopCount+1;
      System.out.println(v_loopCount);
      System.out.println("Trying to align right!");
      //if (RobotContainer.getRobotID() == Constants.kProductionBotID){
      //  changePowerSetPoints(-DriveConstants.kProdBotVisionAlignSpeedDefault, DriveConstants.kProdBotVisionAlignSpeedDefault);
      //}
      //else{
      //  changePowerSetPoints(-DriveConstants.kVisionAlignSpeedDefault, DriveConstants.kVisionAlignSpeedDefault);
      //}
      
      changePowerSetPoints(-RobotContainer.setMotorSpeed(DriveConstants.kProdBotVisionAlignSpeedDefault, DriveConstants.kVisionAlignSpeedDefault), RobotContainer.setMotorSpeed(DriveConstants.kProdBotVisionAlignSpeedDefault, DriveConstants.kVisionAlignSpeedDefault));
      if(v_loopCount>=150){
        System.out.println("Alignment Aborted");
        v_loopCount = 0;
        v_visionOverride = 1;
      }
    }
  }
  public void visionDistanceArea(){
    if (v_limeLightArea > c_VisionAreaTarget){
      //if (RobotContainer.getRobotID() == Constants.kProductionBotID){
      //  changePowerSetPoints(-DriveConstants.kProdBotVisionForwardSpeedDefault, -DriveConstants.kProdBotVisionForwardSpeedDefault);
      //}
      //else{
      //  changePowerSetPoints(-DriveConstants.kVisionForwardSpeedDefault, -DriveConstants.kVisionForwardSpeedDefault);
      changePowerSetPoints(-RobotContainer.setMotorSpeed(DriveConstants.kProdBotVisionForwardSpeedDefault, DriveConstants.kVisionForwardSpeedDefault), -RobotContainer.setMotorSpeed(DriveConstants.kProdBotVisionForwardSpeedDefault, DriveConstants.kVisionForwardSpeedDefault));
      }
    
    if (v_limeLightArea < c_VisionAreaTarget){
      //if (RobotContainer.getRobotID() == Constants.kProductionBotID){
      //  changePowerSetPoints(DriveConstants.kProdBotVisionForwardSpeedDefault, DriveConstants.kProdBotVisionForwardSpeedDefault);
      //}
      //else{
      //  changePowerSetPoints(DriveConstants.kVisionForwardSpeedDefault, DriveConstants.kVisionForwardSpeedDefault);
      //}
      changePowerSetPoints(RobotContainer.setMotorSpeed(DriveConstants.kProdBotVisionForwardSpeedDefault, DriveConstants.kVisionForwardSpeedDefault), RobotContainer.setMotorSpeed(DriveConstants.kProdBotVisionForwardSpeedDefault, DriveConstants.kVisionForwardSpeedDefault));
    }
  }


  public double getRightPosition(){
    return ((BackRightMotor.getSelectedSensorPosition() - v_zeroRightPosition)/c_encoderConversion)*(Math.PI*6);
  }
  
  public double getRightEncoderValue(){
      return BackRightMotor.getSelectedSensorPosition();
  }
  //Distance is in Inches
  public boolean encoderFinish( double distance){
    //return distance <= (Math.abs(getLeftPosition()));
    double average = (Math.abs(getLeftPosition()) + Math.abs(getRightPosition())) / 2;
    return distance <= average;
  }
  
  public double rotateRobot(double m_angle){
    return (((42*Math.PI)*(m_angle/360))*.5);
    //.769 is a fudge factor for carpet to make the calculation work
  }
  //Auto
  public void changePowerSetPoints(double setPointLeft, double setPointright){
    v_setPointLeft = setPointLeft;
    v_setPointRight = setPointright;
  }
  
  public void driveModePowerSetPoint() {
    v_driveMode = c_modeSetPoint;
  }
  public void driveAuto(){
    differentialDrive(-v_setPointLeft, -v_setPointRight);
  }
 //Teleop
  public void driveModeTeleop() {
    v_driveMode = c_modeTeleop;
  }
  public void driveTeleop(){
    differentialDrive((RobotContainer.getLeftSpeed()*RobotContainer.getDriveSpeedAdjust()), (RobotContainer.getRightSpeed()*RobotContainer.getDriveSpeedAdjust()));
  }


    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      
      //Sets drive mode
      if (v_driveMode == c_modeTeleop) {
       driveTeleop();
      } else {
      driveAuto();
      
      }
      updateLimeLight();
      
      SmartDashboard.putNumber("LimelightX", v_limeLightX);
      SmartDashboard.putNumber("LimelightY", v_limeLightY);
      SmartDashboard.putNumber("LimelightArea", v_limeLightArea);
      SmartDashboard.putNumber("Sonar Voltage", sonarSensor.getAverageVoltage());
      SmartDashboard.putNumber("Sonar Distance in Inches", sonarSensor.getAverageVoltage()/DriveConstants.sonarConversionFactor);
      System.out.println(RobotContainer.setMotorSpeed(DriveConstants.kProdBotVisionForwardSpeedDefault, DriveConstants.kVisionForwardSpeedDefault));
    }
  }