/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.SocketVision;
import frc.robot.util.SocketVisionWrapper;
import frc.robot.util.ThrowerLUT;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants;

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
    a controller - joystick(drive) and button (ThrowToLlTargetCommand)
    */
    private ThrowerSubsystem m_subsystem;
    private SwerveDriveSubsystem m_swerve;

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

}