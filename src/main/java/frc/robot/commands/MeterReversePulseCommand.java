/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MeterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class MeterReversePulseCommand extends CommandBase {
  //private final ColorMatch m_colorMatcher = new ColorMatch();

  private MeterSubsystem m_meter;
  private final Timer m_timer;
  private double m_time;

  /**
   * Runs the intake motor in reverse for a specific duration.
   * Motor speed and duration are read from the intake subsystem.
   */
  public MeterReversePulseCommand(MeterSubsystem meter) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Requires the ControlPanel Subsystem
    addRequirements(meter);

    m_meter = meter;
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_time = m_meter.getReversePulse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * Run the intake motor
     */
    m_meter.setMotor(m_meter.getReverseSpeed());
  }

  // Called once the command ends or is interrupted.
  // stop the motor
  @Override
  public void end(boolean interrupted) {
    m_meter.setMotor(0.0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(m_time);
  }
}
