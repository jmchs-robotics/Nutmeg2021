/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.SocketVision;
import frc.robot.util.SocketVisionWrapper;
import frc.robot.util.ThrowerLUT;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants;

// switching to Limelight 3/9
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class ThrowToLlTargetCommand extends CommandBase {
  private ThrowerSubsystem m_subsystem;
  private SocketVisionWrapper m_vision;
  private SwerveDriveSubsystem m_swerve;

  private double setpoint = 0;

  // switching to Limelight 2/22/21
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry tv = table.getEntry("tv");
  

  /**
   * Creates a new ThrowToTarget.
   * Reads the distance to the RFT, from the Vision Coprocessor.
   * Looks up the desired RPM based on the distance, using the ThrowerLUT.
   * Sets the desired RPM of the thrower.
   * Assumes the green LED has already been turned on 
   *   and that the Vision Coprocessor has already been commanded to track the RFT.
   */
  public ThrowToLlTargetCommand(ThrowerSubsystem thrower, SwerveDriveSubsystem swerve, SocketVisionWrapper vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(thrower);

    m_subsystem = thrower;
    m_vision = vision;
    m_swerve = swerve;

    SmartDashboard.putNumber("ThrowToLlTargetCommand distance from Limelight", -111);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getNumber(3);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ll_dist = tx.getDouble(0); // switching to Limelight 3/9

    if( Constants.ThrowerPIDs.TUNE) {
      SmartDashboard.putNumber("ThrowToLlTargetCommand distance from Limelight", ll_dist);
    }
    // Make sure that the vision data is valid
    //if(m_vision.get().get_direction() != SocketVision.NADA){
      double x = 1;
      if( Constants.ThrowerVision.ADAPT_SPEED_TO_POSE_ANGLE) {
        // coprocessor computes distance as inverse of width of target
        // if robot is at an angle (i.e. not straight on) it will think it's farther than it is
        // by a factor of the cos(angle from straight on), i.e. the projection of the target
        x = Math.cos( Math.toRadians(m_swerve.getGyroAngle())); 
      }
      setpoint = -ThrowerLUT.distanceToRPMs( ll_dist * x);
      if( Constants.ThrowerPIDs.TUNE) {
        SmartDashboard.putNumber("ThrowToLlTargetCommand setpoint from Limelight", setpoint);
      }
  //  }
   // else {
      //  setpoint = -ThrowerLUT.DEFAULT_RPM;
   // }
    
    m_subsystem.setThrowerSpeed(0); // setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getNumber(1);
    m_subsystem.stopThrower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}