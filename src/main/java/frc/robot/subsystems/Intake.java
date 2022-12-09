package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeObjects.*;

public class Intake extends SubsystemBase{

    CANSparkMax m_intakeMotor;
    DoubleSolenoid m_intakeSolenoid;

    public Intake() {
        m_intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
        m_intakeSolenoid = new DoubleSolenoid(intakeDSID, intakeDSForward, intakeDSReverse);
    }

    public void setIntake(double speed) {
        m_intakeMotor.set(speed);
    }

    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    public void intakeOut() {
        m_intakeSolenoid.set(Value.kReverse);
    }

    public void intakeIn() {
        m_intakeSolenoid.set(Value.kForward);
    }

    public void intakeOff() {
        m_intakeSolenoid.set(Value.kOff);
    }
}