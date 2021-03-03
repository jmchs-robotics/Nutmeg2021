/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.commands;
 
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MeterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
public class DefaultMeterCommand extends CommandBase {
 
  private XboxController m_controller = new XboxController(1);
  private final MeterSubsystem m_meter;
  
  /**
   * Creates a new DefaultIntakeCommand.
   */
  public DefaultMeterCommand( MeterSubsystem subsystem, XboxController stick) {
    m_meter = subsystem;
    m_controller = stick;
 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    System.out.println("creating ");
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
   // final double yAxis = m_controller.getY(Hand.kLeft);
    final double yAxis = m_controller.getY(Hand.kRight);
    
    //final double xAxis = m_controller.getX(Hand.kLeft);
    // System.out.println("1");

    // run the motor, if the joystick is past a deadband
    if(yAxis >= 0.2){
    //     new IntakeRecieveCommand(m_intake);
    m_meter.setMotor( yAxis);
    // System.out.println("2");
    } else if(yAxis < -0.2){
      m_meter.setMotor( yAxis);
      //new IntakeReversePulseCommand(m_intake);
      // System.out.println("3");
     }
     else {
       m_meter.stopMotor();
     }
 
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_meter.stopMotor();
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
 
 

