/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VBeltSubsystem;


public class SetVBeltCommand extends CommandBase {
  private VBeltSubsystem m_subsystem;
  private double m_speedL = 0;
  private double m_speedR = 0;
  /**
   * Sets the VBelt Motors speed inbetween the scale of -1 to 1.
   * @param speedL is the speed of the left motor on the VBelt
   * @param speedR is the speed of the right motor on the VBelt
   */
  public SetVBeltCommand(VBeltSubsystem subsystem, double speedL, double speedR) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    m_subsystem = subsystem;
    m_speedL = speedL;
    m_speedR = speedR;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setMotor(m_speedR, m_speedL);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotor(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
