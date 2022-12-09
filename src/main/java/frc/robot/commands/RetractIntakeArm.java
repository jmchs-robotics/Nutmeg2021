package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RetractIntakeArm extends CommandBase{

    Intake m_subsystem;

    public RetractIntakeArm(Intake subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_subsystem.intakeIn();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.intakeOff();
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
