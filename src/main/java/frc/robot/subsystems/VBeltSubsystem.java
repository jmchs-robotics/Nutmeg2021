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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.VBeltMotors;



public class VBeltSubsystem extends SubsystemBase {
  private WPI_VictorSPX m_motor_right;
  private WPI_VictorSPX m_motor_left;

  private double m_forwardSpeed;
  private double m_reverseSpeed;
  private double m_reversePulse;
  public static int m_int = 0;

  private DifferentialDrive m_drive;

  /**
   * Creates a new VBeltSubsystem.
   */
  public VBeltSubsystem() {
    m_motor_right = new WPI_VictorSPX(VBeltMotors.VBeltMotorRightID);
    m_motor_left = new WPI_VictorSPX(VBeltMotors.VBeltMotorLeftID);
    m_forwardSpeed = VBeltMotors.forwardSpeed;
    m_reverseSpeed = VBeltMotors.reverseSpeed;
    m_reversePulse = VBeltMotors.reversePulse;

    m_drive = new DifferentialDrive(m_motor_right, m_motor_left);
    
    SmartDashboard.putNumber("class created", m_int);

     if (VBeltMotors.TUNE){
      SmartDashboard.putNumber("VBelt Right Motor Output Percent", m_motor_right.getMotorOutputPercent());
      SmartDashboard.putNumber("VBelt Left Motor Output Percent", m_motor_left.getMotorOutputPercent());
      SmartDashboard.putNumber("VBelt Motor Forward Speed", m_forwardSpeed);
      SmartDashboard.putNumber("VBelt Motor Reverse Speed", m_reverseSpeed);
      SmartDashboard.putNumber("VBelt Motor Reverse Pulse Time", m_reversePulse);
    } 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if (VBeltMotors.TUNE){
      double fs, rs, rp;
      SmartDashboard.putNumber("VBelt Right Motor Output Percent", m_motor_right.getMotorOutputPercent());
      SmartDashboard.putNumber("VBelt Left Motor Output Percent", m_motor_left.getMotorOutputPercent());
      fs = SmartDashboard.getNumber("Intake Motor Forward Speed", 0);
      rs = SmartDashboard.getNumber("Intake Motor Reverse Speed", 0);
      rp = SmartDashboard.getNumber("Intake Motor Reverse Pulse Time", 0);

      if( fs != m_forwardSpeed) {
        m_forwardSpeed = fs;
          setMotor(fs, fs);
      }
      if( rs != m_reverseSpeed) {
        m_reverseSpeed = rs;
          setMotor(rs,rs);
      }
      if( rp != m_reversePulse) {
        m_reversePulse = rp;
      }
      
    } 
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

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  /*public void arcadeDrive() {
		//robotDrive.arcadeDrive( Robot.oi.driverJoystick.getX(), Robot.oi.driverJoystick.getY()); feels like 90 deg off
		robotDrive.arcadeDrive(-Robot.oi.driverJoystick.getY(), Robot.oi.driverJoystick.getX());
	}*/

}
