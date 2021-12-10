/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.commands;
 
import frc.robot.subsystems.NewIntake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
/**
 * Using the VBelt Subsystem
 */
public class DefaultNewIntakeCommand extends CommandBase {
 
  private final NewIntake m_subsystem;
  private XboxController m_stick = new XboxController(1);
  
  /**
   * Creates a new DefaultIntakeCommand.
   */
  public DefaultNewIntakeCommand (NewIntake subsystem, XboxController stick) {
    m_subsystem = subsystem;
    m_stick = stick;
    //SmartDashboard.putString("cheeeeee", "6");
 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  private double deadband (double input){
    if(Math.abs(input) < 0.05) return 0;
    return input;
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double speed = m_stick.getY(Hand.kRight);
    
    //SmartDashboard.putNumber("Check",forward);

    speed *= deadband(speed);
    //backward *= deadband(backward);

    m_subsystem.tankDrive(speed, -speed);
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Try to bring the robot to a dead stop before starting the next command
    m_subsystem.stopMotor();
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
 
 

