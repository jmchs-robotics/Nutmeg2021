/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.ArmMotor;
import frc.robot.Constants.IntakeActuators;
import frc.robot.Constants.VBeltMotors;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



public class ArmSubsystem extends SubsystemBase {
  private WPI_VictorSPX m_motor_right;

  private double m_forwardSpeed;
  private double m_reverseSpeed;
  private double m_reversePulse;
  public static int m_int = 0;

  /**
   * Creates a new VBeltSubsystem.
   */
  public ArmSubsystem() {
    m_motor_right = new WPI_VictorSPX(ArmMotor.ArmMotorID);
    m_forwardSpeed = IntakeActuators.forwardSpeed;
    m_reverseSpeed = IntakeActuators.reverseSpeed;
    m_reversePulse = IntakeActuators.reversePulse;
    
    SmartDashboard.putNumber("class created", m_int);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  /**
   * Set the VBelt motors to a speed between -1 and 1.
   * @param speed
   */
  public void setMotor(double speedRight){
    m_motor_right.set(ControlMode.PercentOutput,speedRight);
  }

  /**
   * Set the VBelt Motors to 0
   */
  public void stopMotor (){
    m_motor_right.set(ControlMode.PercentOutput, 0.0);
  }

  public void motorForward() {
    setMotor(m_forwardSpeed);
  }

  public void motorReverse() {
    setMotor( m_reverseSpeed);
  }

  public double getReversePulse() {
    return m_reversePulse;
  }

  public double getReverseSpeed() {
    return m_reverseSpeed;
  }

  /*
  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }
  */
}
