/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Controller Mapping
    static final int kOperController = 1; // USB
    static final int kDriverController = 0; // USB
    //Robot ID
    public static final int kRobotIDChecker = 0; //AIO
    public static final int kProductionBotID = 0; 
    public static final int kBackupBotID = 1;

    public static final class DriveConstants {
        //Motor Mapping
        public static final int kFrontLeftMotor = 2;
        public static final int kBackLeftMotor = 4;
        public static final int kFrontRightMotor = 1;
        public static final int kBackRightMotor = 17;
        //Auto Move Constants
        public static final double kAutoForwardEncoderDistance = 144; //inches!
        public static final double kAutoBackwardEncoderDistance = 144; 
        public static final int kTestSwitch = 10;
        //Vision Constants
        public static final double kVisionAreaTarget = 1.4;
        //Sensor Constants
        public static final int kOpticalPort = 9; //DIO
        public static final int kSonarPort = 1;
        public static final double sonarConversionFactor = 4.883; //Converts voltage to distance in inches. MAKE THIS FIT CONVENTIONS
        //Drive Speeds
        //This is for the backup bot (Default)
        public static final double kVisionAlignSpeedDefault = .5; 
        public static final double kVisionForwardSpeedDefault = .3850; 
        //This is for the production bot
        public static final double kProdBotVisionAlignSpeedDefault = .50; //These are placeholder values as the actual bot is not yet built
        public static final double kProdBotVisionForwardSpeedDefault = .50; //These are placeholder values as the actual bot is not yet built
    }

    public static final class ShooterConstants {
        //Motor Mapping
        public static final int kShooterMotor = 11;
        public static final int kAimMotor = 12; 
        //Encoder Constants
        public static final int kShooterEncoderChannel1 = 0; //Placeholder value DIO
        public static final int kShooterEncoderChannel2 = 1; //Placeholder value DIO
        public static final int kAimEncoder1 = 2;
        public static final int kAimEncoder2 = 3;
        //Shooter Speed Constants
        public static final double kShooterStop = 0.0;
        public static final double kShooterHalf = 0.5;
        public static final double kShooterFull = 0.8;
        //DIO
        public static final int kAimLimitSwitch = 4;
        
        public static final double kstartingAim = 260;
        public static final double kdeltaAimSecond = 380;
        public static final double kdeltaAimThird = 500;
        public static final double kdeltaAimFar = 500;
    }

    public static final class PickupConstants {
        //Motor Mapping
        public static final int kPickupMotorTop = 6; //CAN 6
        public static final int kPickupMotorBottom = 10; //CAN 10
        public static final int kPickupMotorArm = 3;
    }

    public static final class GateConstants{
        //Gate Mapping
        public static final int Gate1Servo = 0; // PWM
        public static final int Gate2Servo = 1; //PWM
        public static final int Ball1Sensor = 7; // DIO
        //Gate 1
        public static final double gate1Down = 1.0;
        public static final double gate1Half = 0.75;
        public static final double gate1Mid = 0.55;
        public static final double gate1Max = 0.15;
        //Gate 2
        public static final double gate2Down = 1.0;
        public static final double gate2Half = 0.75;
        public static final double gate2Mid = 0.55;
        public static final double gate2Max = 0.15;
        //Gate Timing Constants
        public static final double gateUpTime = 0.5;
        public static final double gateDownTime = 1.5;
		public static final double ShooterSpinUpTime = 2.5;
    }
    public static final class BeltConstants{
        //Talon Mapping
        public static final int kFrontBeltMotor = 21; 
        public static final int kBackBeltMotor = 22; 
        public static final int kBottomBeltMotor = 23; 

        public static final double kBeltForwardSpeed = -.45; 
        public static final double kBeltBackwardSpeed = .2; 

        public static final int kBallReadySensor = 5;
    }
   
}
/*DigitalInput/DigitalOutput Addressing
NAVX-MXP PORT	MXP PIN NUMBER	ROBORIO CHANNEL ADDRESS
0	DIO0	10
1	DIO1	11
2	DIO2	12
3	DIO3	13
4	DIO8	18
5	DIO9	19
6	DIO10	20
7	DIO11	21
8	DIO12	22
9	DIO13	23*/