package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.util.*;
//import jdk.vm.ci.code.InstalledCode;
// import sun.tools.tree.InstanceOfExpression;
// import jdk.vm.ci.code.InstalledCode;
import frc.robot.commands.*;
import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.SwerveDriveModule;

/** 
 * Point all the wheels toward a given angle.  Don't drive anywhere or move the chassis at all.
 */
public class Paths { // extends CommandBase {

    //private SwerveDriveSubsystem m_swerve;
    SwerveDriveSubsystem m_swerve;
    ThrowerSubsystem m_Thrower;
    HopperSubsystem m_Hopper;
    IntakeSubsystem m_Intake;
    SocketVisionSendWrapper sender_;
    SocketVisionWrapper rft_;

    public Paths( SwerveDriveSubsystem swerve, ThrowerSubsystem thrower, HopperSubsystem hopper, IntakeSubsystem intake, SocketVisionSendWrapper sender, SocketVisionWrapper rft) {
        m_swerve = swerve;
        m_Thrower = thrower;
        sender_ = sender;   
        rft_ = rft;     
        m_Hopper = hopper;   
        m_Intake = intake;
    }


    /**
     * test stuff
     * @return
     */
    public Command PathTestCommand() {
      return new SequentialCommandGroup(
        new DriveForTime2910Command(m_swerve, 0.15, .2, 0)
      );
    }

    /**
     * test stuff
     * @return
     */
    public Command TestUnload() {
      return new UnloadCommand(m_swerve, m_Thrower, m_Hopper, sender_, rft_, 1);
    }

    /**
     * Go from fence back and left to mid/back scoring position, staying right of another robot striaght in front of goal 
     * aim, score
     */
    public Command Path1Command() {
      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::holonomicDriveToZero, m_swerve),
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // tell vision coprocessor to track the RFT
        new SetWheelAngleCommand( m_swerve, -18-90),  // point the wheels in the direction we want to go
        new WaitCommand( 0.2), // 0.2), // give the drivetrain a chance to respond to the SetWheelAngle command
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), // test doing DriveForDist at slow speed
        new DriveForTime2910Command(m_swerve, 0.8, .2, -.5), // this drives pretty close to -57, -12
        //new DriveForDist2910Command( m_swerve, -57, -12), // go to destination 94 - (25+6.5)/2 - 28
        new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), // put DriveForDist at regular speed
        new WaitCommand( 1), // give vision coprocessor a chance to find the target
        new SetPoseAngle2910Command( m_swerve, 10),
        new WaitCommand( 1), // give vision coprocessor a chance to find the target
        // TODO: UnloadCommand().  remove VisionAim and any last WaitCommand()
        new VisionAimGyroCommand( m_swerve, rft_), // aim the robot
        new ParallelRaceGroup(
          new ThrowToTargetCommand(m_Thrower, m_swerve, rft_),  // never ends
            new SequentialCommandGroup( 
              new WaitCommand(2),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1)
            )
          ),
        //new SetThrowerSpeedCommand(m_Thrower, 0),
              
        // very last thing
        new InstantCommand(m_Thrower::turnOffLED, m_Thrower), // Turn off green LED
        new InstantCommand(m_swerve::setBrakeOff, m_swerve)
      );
    }


    /**
     * Go from fence forward and left to close scoring position, staying right of another robot striaght in front of goal 
     * aim, score
     */
    public Command Path2Command() {
      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::holonomicDriveToZero, m_swerve),
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // tell vision coprocessor to track the RFT
        new SetWheelAngleCommand( m_swerve, 18-90),  // point the wheels in the direction we want to go
        new WaitCommand( 0.2), // 0.2), // give the drivetrain a chance to respond to the SetWheelAngle command
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), // test doing DriveForDist at slow speed
        new DriveForTime2910Command(m_swerve, 1., -.2, -.5), // this drives pretty close to -57, 12
        //new DriveForDist2910Command( m_swerve, -57, 12), // go to destination 94 - (25+6.5)/2 - 28
        new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), // put DriveForDist at regular speed
        new WaitCommand( 1), // give vision coprocessor a chance to find the target
        new SetPoseAngle2910Command( m_swerve,0),
        new WaitCommand( 1), // give vision coprocessor a chance to find the target
        // TODO: UnloadCommand().  remove VisionAim and any last WaitCommand()
        new VisionAimGyroCommand( m_swerve, rft_), // aim the robot
        new ParallelRaceGroup(
          new ThrowToTargetCommand(m_Thrower, m_swerve, rft_),  // never ends
            new SequentialCommandGroup( 
              new WaitCommand(2),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1)
            )
          ),
        //new SetThrowerSpeedCommand(m_Thrower, 0),
              
        // very last thing
        new InstantCommand(m_Thrower::turnOffLED, m_Thrower), // Turn off green LED
        new InstantCommand(m_swerve::setBrakeOff, m_swerve)
      );
    }

    /**
     * after Path1, drive into our own trench and retrieve first 3 balls.  We would only do this if our starting position is nearest the trench among our alliance.
     * — drive completely on encoders
     * — someday drive on Vision of balls
     * — end command via encoders, or someday by counting balls
     * -- Someday set pose angle absolute, then strafe to near the first ball, then a command to drive by vision toward the ball until the ball is acquired or we hit something (accelerometer impact) command
     * return to target range and shoot them
     * -- need to drive to a field position, probably only one move (i.e. direct angle) or maybe 2, then rotate to straight at target, then setPoseAngleToVisionRFT
     * @return
     */
    public Command PathCCommand() {
      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new SetWheelAngleCommand( m_swerve, Math.atan2( 57-28, -(86-12-(34+6.5)/2))),  // point the wheels in the direction we want to go
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), 
        new DriveForDist2910Command( m_swerve, 57-28, -(86-12-(34+6.5)/2), "0"), // go to front side of trench, aligned with balls
        new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), // put DriveForDist at regular speed
        new SetPoseAngle2910Command(m_swerve, -90), // point intake at the balls by turning left 90 degrees
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), 
        new InstantCommand(m_Intake::lowerIntake, m_Intake),
        new ParallelRaceGroup( 
          new IntakeAdvDaisyCommand(m_Intake, m_Hopper), // intake and auto-advance the Daisy
          new DriveForDist2910Command( m_swerve, 0, -(108+12), "0") // drive through 3 balls
        ),
        new InstantCommand(m_Intake::raiseIntake, m_Intake),
        new SetWheelAngleCommand( m_swerve, Math.atan2( -(57-28), 86-12-(34+6.5)/2 + 108+12)),  // point the wheels in the direction we want to go
        new DriveForDist2910Command( m_swerve, -(57-28), 86-12-(34+6.5)/2 + 108+12, "0"),
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // tell vision coprocessor to track the RFT
        new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), // put DriveForDist at regular speed
        new SetPoseAngle2910Command(m_swerve, -5),
        
        new ParallelCommandGroup( // waits for both to end
                  new SpinUpThrowerCommand(m_Thrower, m_swerve, rft_),  // set thrower speed to vision distance, end when it's there
                  new VisionAimGyroCommand( m_swerve, rft_) // aim the robot
        ),
        new ParallelRaceGroup( // ends when first command ends
          new ThrowToTargetCommand(m_Thrower, m_swerve, rft_),  // never ends
          new SequentialCommandGroup(
            new BumpHopperCommand(m_Hopper),
            new MoveHopperCommand(m_Hopper, 6)
          )
        ),
        new SetThrowerSpeedCommand(m_Thrower, 0),
    
        // very last thing
        new InstantCommand(m_Thrower::turnOffLED, m_Thrower), // Turn off green LED
        new InstantCommand(m_swerve::setBrakeOff, m_swerve)

      );
    }
    
    /**
     * PathG
     * Start against left fence, with front of robot against the fence 
     * and intake toward our driver station (-90 degrees from front-facing).
     * Robot on our driver's side of initiation line
     */
    /*
    PathG
    // change field orientation offset by -90
    // angle wheels at -95, to drive mostly left and a little backwards (robot oriented), i.e. backwards and a little right (field oreinted)
    // 130 = initiation line to table edge.  31.5 = width of robot.  12.5 = beater bar outside the robot + buffer distance
    // 8 = (56 - 40.5) / 2 = (table width - robot length) / 2
    // field oriented, drive at full speed backwards (130 - 31.5 - 12.5) and right 8
    // intake down
    // turn robot -20 degrees (robot oriented), which makes ball nearer fence 3.4" closer to the beater bar and the ball closer to the center of the field 3.4" farther from the beater bar
    // angle wheels at -70
    // intake on (inward)
    // 8 = buffer distance. 7 = ball diameter. 7 = 2nd ball diameter.
    // drive at slow speed backwards ( 8+7+7) and zero to the right
    // pause, to intake second ball
    // ange wheels at 
    // 162 = half field width. 8 = how far we already moved from fence. 20 = half the robot's length. 
    // 65 = center of field to center of goal. 20 = half the robot's lenght. 14 = safety distance if alliance robot is aligned center of goal and still on initiation line.
    // drive at full speed forward ((130 - 31.5 - 12.5) + ( 8+7+7) - 6) and right (162-8-20+65-20-14)
    // turn robot +20 degrees (robot oriented) to face forwards
    // VisionAim and Unload
    */

    /**
     * Autonomus Paths for the Auto Nav Challenge
     * They have not been checked by the robot
     * @return
     */

    public Command PathBarrelCommand() {
      int cmd_idx = 0;
      double w = 0.25;
      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        //new SetWheelAngleCommand( m_swerve, Math.atan2( 57-28, -(86-12-(34+6.5)/2))),  // point the wheels in the direction we want to go
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), 
        
        //Update 1/29 cut distance in half to test
        // Update 2/16 The Wheels need to be set to a specific angle before the go and by adding the wait command, we have busted the ghost
        // The 2/16 Update needs to be added to the other autoPaths
        //On the image in the manual referencing the path, north means up, south means down, east means right/foward, and west means left/backward
        //DriveforDist2910Command(Subsystem drivetrain, distRight, distFoward)
        //Going around Nav Point D5
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, 135, Integer.toString( cmd_idx++ )), //Move east 135" // cmd_idx = 0
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 36, 0, Integer.toString( cmd_idx++ )), //Move south 36"
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -45, Integer.toString( cmd_idx++ )), //Move west 45" 
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -66, 0, Integer.toString( cmd_idx++ )), //Move north 66"
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        //Going around Nav Point B8
        new DriveForDist2910Command(m_swerve, 0, 174, Integer.toString( cmd_idx++ )), //Move east 174" 
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -60, 0, Integer.toString( cmd_idx++ )), //Move north 60"
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -45, Integer.toString( cmd_idx++ )), //Move west 45"
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 150, 0, Integer.toString( cmd_idx++ )), //Move south 150"
        //Going around Nav Point D10 and to Finish Zone
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, 120, Integer.toString( cmd_idx++ )), //Move east 120"
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -60, 0, Integer.toString( cmd_idx++ )), //Move north 60"
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -294, Integer.toString( cmd_idx++ ))); //Move west 294" // cmd_idx = 10
    }

    public Command PathBounceCommand() {
      int cmd_idx = 0;
      double w = 0.25;

      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        //new SetWheelAngleCommand( m_swerve, Math.atan2( 57-28, -(86-12-(34+6.5)/2))),  // point the wheels in the direction we want to go
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), 
        //Starting in the middle with bumper against the most left line of the Start Zone
        //x, y (inches) ( + == right, + == up )
        //SetWheelAngleCommand(m_swerve, Math.toDegress(Math.atan2(y,x)))
        //DriveForDist2910Command(m_swerve, x,y, Interger.toString (cmd_indx++))
        //Change in x ==> Set angle to 0
        //Change in y ==> Set angle to 90
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(10.5, 45))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 45, 10.5, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, 35.5, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w), 
        new DriveForDist2910Command(m_swerve, 0, -35.5, Integer.toString( cmd_idx++ )),
        //First Bounce ("bounces"over points B4, B5, and D5??)
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(-20, 30))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 30, -20, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -50, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 60, 0, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0,  .5, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -100.5, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 90, 0, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, 100.5, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -40.5, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 30, 0, Integer.toString( cmd_idx++ )));
    }

    public Command PathSlalomCommand() {
      int cmd_idx = 0;
      double w = 0.25;
      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        //new SetWheelAngleCommand( m_swerve, Math.atan2( 57-28, -(86-12-(34+6.5)/2))),  // point the wheels in the direction we want to go
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), 
        //On the image in the manual referencing the path, north means up, south means down, east means right/foward, and west means left/backward
        //DriveforDist2910Command(Subsystem drivetrain, distRight,, Integer.toString( cmd_idx++ ) distFoward)
        //SetWheelAngleCommand(m_swerve, Math.toDegress(Math.atan2(y,x)))
        //DriveForDist2910Command(m_swerve, x,y, Interger.toString (cmd_indx++))
        //Change in x ==> Set angle to 0
        //Change in y ==> Set angle to 90
        //Start on top left corner of the Start zone
        // Go to above Nav Point D4
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, 60, Integer.toString( cmd_idx++ )), //Move east 60"
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -37, 0, Integer.toString( cmd_idx++ )), //Move north 37"
        //Across Nav Points D4 through D8
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, 180, Integer.toString( cmd_idx++ )), //Move east 180"
        // Around Nav Point D10
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(60, 37))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 37, 60, Integer.toString( cmd_idx++ )), //Move south 37"         ??? 60 --> and east 60"?
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, 45, Integer.toString( cmd_idx++ )), //Move east 45"
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -37, 0, Integer.toString( cmd_idx++ )), //Move north 37"         
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -60, Integer.toString( cmd_idx++ )), //Move west 60"
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(60, 3))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 37, 60, Integer.toString( cmd_idx++ )), //Move south 37"         ??? 60 --> and east 60"?
        // Back across Nav Points D8 to D4new SetWheelAngleCommand(m_swerve, 0),
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -120, Integer.toString( cmd_idx++ )), //Move west 120"
        //Into Finish Zone
        new SetWheelAngleCommand(m_swerve, 0),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -37, 0, Integer.toString( cmd_idx++ )), //Move north 37"
        new SetWheelAngleCommand(m_swerve, 90),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -60, Integer.toString( cmd_idx++ ))); //Move west 60"
    }

    public Command AngledSlalomCommand() {
      
      int cmd_idx = 0;
      double w = 0.25;

      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        //Change in x ==> Set angle to 0
        //Change in y ==> Set angle to 90
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), 
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(58, -34))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -34, 58, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(165, 0))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, 165, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(18, 36))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 36, 18, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(42, 0))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, 42, Integer.toString( cmd_idx++ )), 
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(0, -36))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -36, 0, Integer.toString( cmd_idx++ )), 
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(-45, 0))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -45, Integer.toString( cmd_idx++ )),   
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(-18, 36))),
        new WaitCommand(w),      
        new DriveForDist2910Command(m_swerve, 36, -18, Integer.toString( cmd_idx++ )), 
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(-162, 0))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -162, Integer.toString( cmd_idx++ )), 
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(-58, -34))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -34, -58, Integer.toString( cmd_idx++ )));
    }
    
    public Command AngledBarrelCommand() {
      
      int cmd_idx = 0;
      double w = 0.25;

      //new DriveForDist2910Command(m_swerve, x, y, Integer.toString( cmd_idx++ ))
      //new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(y, x)))

      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), 
        new SetWheelAngleCommand(m_swerve,0),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(133, 0))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, 133, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(0, 36))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 36, 0, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(-45, 0))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -45, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(0, -36))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -36, 0, Integer.toString( cmd_idx++ )), 
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(135, -24))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -24, 135, Integer.toString( cmd_idx++ )), 
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(0, -36))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -36, 0, Integer.toString( cmd_idx++ )),   
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(-45, 0))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -45, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(0, 36))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 36, 0, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(60, 60))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 60, 60, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(45, 0))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, 45, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(0, -36))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, -36, 0, Integer.toString( cmd_idx++ )),
        new SetWheelAngleCommand(m_swerve, Math.toDegrees(Math.atan2(-283, 0))),
        new WaitCommand(w),
        new DriveForDist2910Command(m_swerve, 0, -283, Integer.toString( cmd_idx++ )));
        
    }

    public Command SpeedBounceCommand() {
      int cmd_idx = 0;

      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new SetWheelAngleCommand( m_swerve, Math.atan2( 57-28, -(86-12-(34+6.5)/2))),  // point the wheels in the direction we want to go
        new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), 
        //Starting in the middle with bumper against the most left line of the Start Zone
        //x, y (inches) ( + == right, + == up )
        new DriveForDist2910Command(m_swerve, 45, 10.5, Integer.toString( cmd_idx++ )),
        new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), 
        new DriveForDist2910Command(m_swerve, 0, 35.5, Integer.toString( cmd_idx++ )),
        new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), 
        new DriveForDist2910Command(m_swerve, 0, -35.5, Integer.toString( cmd_idx++ )),
        //First Bounce ("bounces"over points B4, B5, and D5??)
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), 
        new DriveForDist2910Command(m_swerve, 30, -20, Integer.toString( cmd_idx++ )),
        new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), 
        new DriveForDist2910Command(m_swerve, 0, -50, Integer.toString( cmd_idx++ )),
        new DriveForDist2910Command(m_swerve, 60, 0, Integer.toString( cmd_idx++ )),
        new DriveForDist2910Command(m_swerve, 0,  .5, Integer.toString( cmd_idx++ )),
        new DriveForDist2910Command(m_swerve, 0, -100.5, Integer.toString( cmd_idx++ )),
        new DriveForDist2910Command(m_swerve, 90, 0, Integer.toString( cmd_idx++ )),
        new DriveForDist2910Command(m_swerve, 0, 100.5, Integer.toString( cmd_idx++ )),
        new DriveForDist2910Command(m_swerve, 0, -40.5, Integer.toString( cmd_idx++ )),
        new DriveForDist2910Command(m_swerve, 30, 0, Integer.toString( cmd_idx++ )));
    }
}
