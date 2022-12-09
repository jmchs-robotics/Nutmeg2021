package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeBall extends CommandBase{

    Intake m_subsystem;

    public IntakeBall(Intake subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_subsystem.setIntake(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

  @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
