/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Declare inner public classes to segment constants

  /**
   * for vision sockets, comms with Vision Co-processor, and using vision within the rest of the subsystems.
   */
  public static final class Vision {
    public static final boolean SHOW_DEBUG = false;
    public static final long RFT_X_OFFSET = 45; // offset in pixels of vision output vs where we want to point/move the robot
    public static final double RFT_PIXELS_TO_DEGREES = .110755; 
    public static final boolean TUNE = true;
    public static final double RFT_X_OFFSET_LL = 0;  // limelight offset, degrees, 3/9
  }
  
  /**
  * Contains the CAN IDs of the Drivetrian SparkMax motors
  */ 
  public static final class DrivetrainMotors {
    public static final int FRONT_LEFT_DRIVE = 25;
    public static final int FRONT_LEFT_ANGLE = 23;

    public static final int FRONT_RIGHT_ANGLE = 34;
    public static final int FRONT_RIGHT_DRIVE = 32;

    public static final int BACK_LEFT_DRIVE = 26;
    public static final int BACK_LEFT_ANGLE = 24;

    public static final int BACK_RIGHT_ANGLE = 33;
    public static final int BACK_RIGHT_DRIVE = 31;
  }

  public static final class DrivetrainConstants {
    // set for SwerveyJr 191207
    public static final double WHEELBASE = 27.5;
    public static final double TRACKWIDTH = 18.5;
    public static final double WIDTH = 25.75;
    public static final double LENGTH = 28;

    // PID constants for swerve modules
    public static final double ANGLE_kP = 3.0;
    public static final double ANGLE_kI = 0.0;
    public static final double ANGLE_kD = 0.0;    

    // PID constants for whole-drivetrain strafe control
    public static final double STRAFE_kP = 0.01;
    public static final double STRAFE_kI = 0.0;
    public static final double STRAFE_kD = 0.0;

    // PID constants for whole-drivetrain strafe control
    public static final double FORWARD_kP = 0.01;
    public static final double FORWARD_kI = 0.0;
    public static final double FORWARD_kD = 0.0;

    // PID constants for whole-drivetrain strafe control.
    // 0.03, 0, 0.075 are from 2910's pose angle code
    public static final double ROTATION_kP = 0.03;
    public static final double ROTATION_kI = 0.0;
    public static final double ROTATION_kD = 0.0;

    // PID for controlling rotation in DriveForDist2910Command
    // they had set .02, 0, 0 
    public static final double DFD_ROTATION_kP = 0.03; // for next test after 3/1, try drastically reducing from 0.03
    public static final double DFD_ROTATION_kI = 0.0;
    public static final double DFD_ROTATION_kD = 0.0;

    // PID for controlling pose angle in SetPoseAngle2910 and VisionAim* commands.
    public static final double POSE_ANGLE_kP = 0.03;
    public static final double POSE_ANGLE_kI = 0.0;
    public static final double POSE_ANGLE_kD = 0.0;

    public static final boolean TUNE = true;
  }

  /**
   * Constants for the color sensor targets, in RGB arrays
   */
  public static final class ColorTargets {
    public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    public static final double cpSpinnerSpeed = 0.5;
  }
  
  /**
   * Constants for the control panel
   */
  public static final class ControlPanelActuators {
    public static final int victorID = 10;
    public static final int soleniodForward = 0;
    public static final int soleniodBackward = 1;
    public static final Port sensorPort = I2C.Port.kOnboard;
    public static final boolean TUNE = true;
  }

  /**
   * Constants for the climb
   */
  public static final class ClimbConstants {
    public static final int climbVictorID = 8;
    public static final int soleniodForward = 4;
    public static final int soleniodBackward = 5;
    public static final int extendSolenoidForward = 6;
    public static final int extendSolenoidReverse = 7;

    // motor run characteristics
    public static final double forwardSpeed = 0.7;
    public static final double reverseSpeed = -0.7;

    // set to true to put motor speed and other data on the smart dashboard
    public static final boolean TUNE = true;
  }

  // vision coprocessor computes distance as inverse of width of target
  // if robot is at an angle (i.e. not straight on) then the coprocessor will think the target is farther than it is
  // by a factor of the cos(angle from straight on), i.e. the projection of the target
  // set this to true to multiply the distance returned by vision to the angle of the robot's pose,
  // i.e. to scale the distance back down
  public static final class ThrowerVision {
    public static final boolean ADAPT_SPEED_TO_POSE_ANGLE = true;
  }

  public static final class ThrowerMotor {
    public static final int THROWER_MASTER_ID = 14;
    public static final int THROWER_FOLLOWER_ID = 15;
    public static final boolean INVERT_FOLLOWER = false;
	public static final int LED_CHANNEL = 0;
  }

  public static final class ThrowerPIDs {
    public static final double kP = 2e-4;
    public static final double kI = 1e-6;
    public static final double kD = 0.0;
    public static final double FEED_FORWARD = 0.0; // minimum useful 1/18 seems to be 0.001
    public static final double kIz = 0;
    public static final double MIN_OUTPUT = -1;
    public static final double MAX_OUTPUT = 1;
    public static final double GEAR_RATIO_MOTOR_TO_WHEEL = 35.0 / 35.0; // 40T pinion, 48T wheel gear
    public static final int UPDATE_RATE = 5; // msecs  200 Ht update rate leader -> folloer. Default 10ms
    public static final boolean TIME = false;
    public static final boolean TUNE = true;
  }

  public static final class IntakeActuators {
    public static final int intakeVictorID = 9;
    public static final int intakeSoleniodForward = 2;
    public static final int intakeSoleniodBackward = 3;

    // motor run characteristics
    public static final double forwardSpeed = 0.7;
    public static final double reverseSpeed = -0.7;
    public static final double reversePulse = 0.5; 

    // set to true to put motor speed and other data on the smart dashboard
    public static final boolean TUNE = true;
  }

  // Motor ID for the VBelt Subsystem
    public static final class VBeltMotors {
    public static final int VBeltMotorRightID = 49;
    public static final int VBeltMotorLeftID = 50;

    // motor run characteristics
    public static final double forwardSpeed = 0.7;
    public static final double reverseSpeed = -0.7;
    public static final double reversePulse = 0.5;

    // set to true to put motor speed and other data on the smart dashboard
    public static final boolean TUNE = true;
  }


  public static final class HopperConstants {
    public static final int HOPPER_MOTOR_ID = 20;
    public static final double ONE_ROTATION = 4096;
    public static final int ALLOWABLE_ERROR = 0;

    public static final double DAISY_OFFSET = 20 * 4096/360;
    public static final double DARK_THRESH = 3.2; // between 0 (pitch black) and 5 (bright light)
    // number of samples (one every 0.02 seconds) over which we want to 
    //average the photodiode’s input to make sure we only trigger when a ball’s really there
    public static final int PHOTO_NUM_SAMPLES = 12; 
    public static final int sdThrottleReset = 25; // start counting from here to 50 for how often to update the SmartDashboard
    
  }

  public static final class HopperPIDs {
    /**
	  * Which PID slot to pull gains from. Starting 2018, you can choose from
	  * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
    * configuration.
    */
    /**
	  * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	  * now we just want the primary one.
	  */
    public static final int kPIDLoopIdx = 0;

    public static final double kP = 1.6; // 2e-2;
    public static final double kI = 0; // 1e-6;
    public static final double kD = 0.0;
    public static final double kF = 0; // 2e-6;
    public static final double MIN_OUTPUT = -0.3;
    public static final double MAX_OUTPUT = 0.3;
    
	  /**
	  * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	  * report to DS if action fails.
	  */
	  public static final int kTimeoutMs = 0; // 30;
	
    /** 
     * Choose so that Talon does not report sensor out of phase 
    * false for prototyping 1/26/20.  
    * Perhaps set to true when we have the daisy built.
    */
	  public static boolean kSensorPhase = false;

	  /**
	  * Choose based on what direction you want to be positive
	  */
	  public static boolean kMotorInvert = false;

    // set to true to put PID and other data on the smart dashboard
    public static final boolean TUNE = true;
    
  }

  /** 
   * which DIO ports control which LEDs
   */
  public static final class LED {
    public static final int GREEN = 0;
    public static final int SPOTLIGHT = 1;
  }

  /** 
   * autonomous 
   */
  public static final class AUTO {
    public static final double DISTANCE_CHECK_TIME = 0.25;
    public static final boolean TUNE = true;
  }

}
