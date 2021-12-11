/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.IntakeMotors;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



public class NewIntake extends SubsystemBase {
  private WPI_VictorSPX m_motor_right;
  private WPI_TalonSRX m_motor_left;

  private double m_forwardSpeed;
  private double m_reverseSpeed;
  private double m_reversePulse;
  public static int m_int = 0;

  private DifferentialDrive m_drive;

  public NewIntake() {
    m_motor_right = new WPI_VictorSPX(IntakeMotors.IntakeMotorRightID);
    m_motor_left = new WPI_TalonSRX(IntakeMotors.IntakeMotorLeftID);
    m_forwardSpeed = IntakeMotors.forwardSpeed;
    m_reverseSpeed = IntakeMotors.reverseSpeed;
    m_reversePulse = IntakeMotors.reversePulse;

    m_drive = new DifferentialDrive(m_motor_left, m_motor_right);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }

  /**
   * Set the VBelt motors to a speed between -1 and 1.
   * @param speed
   */
  public void setMotor(double speedRight, double speedLeft){
    m_motor_right.set(ControlMode.PercentOutput,speedRight);
    m_motor_left.set(ControlMode.PercentOutput,speedLeft);
  }

  /**
   * Set the VBelt Motors to 0
   */
  public void stopMotor (){
    m_motor_right.set(ControlMode.PercentOutput, 0.0);
    m_motor_left.set(ControlMode.PercentOutput, 0.0);
  }

  public void motorForward() {
    setMotor(m_forwardSpeed, m_forwardSpeed);
  }

  public void motorReverse() {
    setMotor( m_reverseSpeed, m_reverseSpeed);
  }

  public double getReversePulse() {
    return m_reversePulse;
  }

  public double getReverseSpeed() {
    return m_reverseSpeed;
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    m_drive.arcadeDrive(xSpeed, zRotation);
  }

  
  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }
}
