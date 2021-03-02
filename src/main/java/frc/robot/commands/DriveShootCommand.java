/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.JoystickAnalogButton;
import frc.robot.util.SocketVision;
import frc.robot.util.SocketVisionWrapper;
import frc.robot.util.ThrowerLUT;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants;
import frc.robot.Constants.ThrowerMotor;
import frc.robot.commands.autonomous.DriveForDist2910Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveShootCommand extends CommandBase
{
    //shoot for inner port first

    /*
    we need:
    a thrower
    a drivetrain
    a controller - joystick(drive) and left/right trigger (ThrowToLlTargetCommand)
    */
    private ThrowerSubsystem m_subsystem;
    private SwerveDriveSubsystem drivetrain;
    
    private final XboxController m_primaryController = new XboxController(1);
    private final JoystickButton m_primaryController_A = new JoystickButton(m_primaryController,
      XboxController.Button.kA.value);
    private final JoystickAnalogButton m_primaryController_RightTrigger = new JoystickAnalogButton(m_primaryController, Hand.kRight, 0.5);
    
    private SocketVisionWrapper m_vision;

    //start in green zone, human playes use joystick to drive
    //aim itself to shoot - use ThrowToLlTargetCommand to aim and shoot
    //go back to re-introduction zone, let human player put power cells in
    //drive to yellow zone
    //aim itself to shoot
    //go back to re-introduction zone, let human player put power cells in
    //drive to blue zone
    //aim itself to shoot
    //go back to re-introduction zone, let human player put power cells in
    //drive to red zone
    //aim itself to shoot
    //end
    public DriveShootCommand(ThrowerSubsystem subsystem, SwerveDriveSubsystem drivetrain, SocketVisionWrapper vision)
    {
        m_subsystem = subsystem; 
        this.drivetrain = drivetrain;
        m_vision = vision;
    }

    @Override
    public void initialize() 
    {

    }

    @Override
    public void execute()
    {
        m_primaryController_A.whenPressed(
            new DriveForDist2910Command(drivetrain, 0, 50)
        );
        m_primaryController_RightTrigger.whenHeld(
            new ThrowToLlTargetCommand(m_subsystem, drivetrain, m_vision)
        );

    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

}