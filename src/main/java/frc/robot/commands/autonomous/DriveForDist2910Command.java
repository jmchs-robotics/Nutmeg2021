package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.Constants.AUTO;
import frc.robot.Constants.DrivetrainConstants;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class DriveForDist2910Command extends CommandBase {
    private static final double TARGET_DISTANCE_BUFFER = 4; // 2;
    private static final double DISTANCE_CHECK_TIME = 0.25;

    private final SwerveDriveSubsystem drivetrain;
    private final double angle;
    private final double distance;
    private final double distRight, distForward;
    private final PIDController angleErrorController;
    private final Timer finishTimer = new Timer();
    private boolean isTimerStarted = false;

    private double initialDrivetrainAngle = 0;
    private double rotationFactor = 0;

    private double rotationMinOutput = -1;
    private double forwardMaxOutput = 1;
    private double strafeMaxOutput = 1;
    private double rotationMaxOutput = 1;

    private BufferedWriter[] encPosLoggers = new BufferedWriter[4];
    private BufferedWriter[] encVelLoggers = new BufferedWriter[4];
    private int iterCount;

    private boolean isGyroSet = false;

    private String fileID;

    /**
     * Move the robot forward/backwards this many inches. 
     * Field oriented.
     * @param drivetrain 
     * @param distance inches; positive if forwards, negative is backwards.
     */
    public DriveForDist2910Command(SwerveDriveSubsystem drivetrain, double distance) {
        this(drivetrain, 0, distance, "0");
    }

    /**
     * Move the robot this many inches.  Robot maintains its same pose angle.
     * Field oriented.
     * @param drivetrain
     * @param distRight inches positive is to the right; negative is to the left.
     * @param distForward inches positive is forward; negaitve is backwards.
     */
    public DriveForDist2910Command(SwerveDriveSubsystem drivetrain, double distRight, double distForward, String fileID) {
        this.drivetrain = drivetrain;
        this.angle = Math.toDegrees(Math.atan2(distRight, distForward));
        
        this.distRight = distRight; // -distRight;  // 191206 seems like this should not get inverted
        this.distForward = distForward;
        
        this.distance = Math.sqrt(distRight * distRight + distForward * distForward);
        // Calculations done by AngleErrorController are invalid, will be applying in about line 145:
        // 191206 this PID isn't working... probably needs more P.  Original is 0.02, 0, 0
        angleErrorController = new PIDController(DrivetrainConstants.DFD_ROTATION_kP, DrivetrainConstants.DFD_ROTATION_kI, DrivetrainConstants.DFD_ROTATION_kD);
        angleErrorController.enableContinuousInput(0, 360);
        angleErrorController.reset();
        
        this.fileID = fileID;
        /*
         new PIDController(0.02, 0, 0, new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) { }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }

            @Override
            public double pidGet() {
                return drivetrain.getGyroAngle();
            }
        }, output -> {
            rotationFactor = output;
        });
        
        angleErrorController.setInputRange(0, 360);
        angleErrorController.setOutputRange(-0.5, 0.5);
        angleErrorController.setContinuous(true);
        */
        addRequirements(drivetrain);
    }

    public DriveForDist2910Command(SwerveDriveSubsystem drivetrain, double distRight, double distForward, double poseAngle, String fileID) {
        this.drivetrain = drivetrain;
        this.angle = poseAngle;
        
        this.distRight = distRight; // -distRight;  // 191206 seems like this should not get inverted
        this.distForward = distForward;
        
        this.distance = Math.sqrt(distRight * distRight + distForward * distForward);
        // Calculations done by AngleErrorController are invalid, will be applying in about line 145:
        // // 191206 this PID isn't working... probably needs more P.  Original is 0.02, 0, 0
        angleErrorController = new PIDController(DrivetrainConstants.DFD_ROTATION_kP, DrivetrainConstants.DFD_ROTATION_kI, DrivetrainConstants.DFD_ROTATION_kD);
        angleErrorController.enableContinuousInput(0, 360);
        angleErrorController.reset();

        isGyroSet = true;

        this.fileID = fileID;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        resetPID();
        finishTimer.stop();
        finishTimer.reset();
        isTimerStarted = false;

        if (isGyroSet) {
            initialDrivetrainAngle = angle;
        }
        else {
            initialDrivetrainAngle = drivetrain.getGyroAngle();
        }
        
        angleErrorController.setSetpoint(initialDrivetrainAngle);

        //Calculations done by AngleErrorController are invalid, will be applying in about line 145:
        // angleErrorController.enable();

        for (int i = 0; i < 4; i++) {
            drivetrain.getSwerveModule(i).setTargetAngle(angle + initialDrivetrainAngle); // drivetrain.getGyroAngle());
            drivetrain.getSwerveModule(i).zeroDistance();
            drivetrain.getSwerveModule(i).setTargetDistance(distance);
        }

        iterCount = 0;
        
        if(AUTO.LOG) {
            for (int i = 0; i < 4; i++) {
                try {
                    // 191206 take space out of filename
                    encPosLoggers[i] = Files.newBufferedWriter(Paths.get(String.format("/home/lvuser/encPos%d_%s.csv", i, fileID)));
                    encPosLoggers[i].write( "count, millisecs, encoderPos, driveDist, gyroAngle, rotation, newModuleAngle, oldModuleAngle, x\n");
                    //encVelLoggers[i] = Files.newBufferedWriter(Paths.get(String.format("/home/lvuser/encVel%d_%s.csv", i, fileID)));
                } catch (IOException e) {
                    encPosLoggers[i] = null;
                    encVelLoggers[i] = null;
                }
            }
    
        }        

        if( AUTO.TUNE) {
            SmartDashboard.putNumber("Drive Distance Forward", distForward);
            SmartDashboard.putNumber("Drive Distance Right", distRight);
        }
    }

    @Override
    public void execute() {
        double forwardFactor = distForward / distance;
        double strafeFactor = -distRight / distance;
        double g = drivetrain.getGyroAngle(); // measurement
        // setting the inital drive angle minus the gyro angle
        double x = (initialDrivetrainAngle - g) % 360;
        if( x > 180) {
            x -= 360;
        }
        else if (x < -180){
            x += 360;
        }
        double rotation =  x  * (DrivetrainConstants.DFD_ROTATION_kP); //angleErrorController.calculate(x); //x * (DrivetrainConstants.DFD_ROTATION_kP); 
        
        // rotation = Math.min( -0.5, Math.max( 0.5, rotation));  // clamp

        double[] moduleAngles = drivetrain.calculateSwerveModuleAngles(forwardFactor, strafeFactor, rotation);  // -rotationFactor); // 191206

        for (int i = 0; i < 4; i++) {
            drivetrain.getSwerveModule(i).setTargetAngle(moduleAngles[i]); 
            if(AUTO.LOG)
            {
                try {
                    // 191206 also save the time in milliseconds, and the driveDistance
                    
                    encPosLoggers[i].write( String.format("%d, %d, %f, %f, %f, %f, %f, %f, %f\n",
                        iterCount, 
                        System.currentTimeMillis(),
                        //Math.abs(drivetrain.getSwerveModule(i).getDrivePosition()),
                        drivetrain.getSwerveModule(i).getDrivePosition(), 
                        drivetrain.getSwerveModule(i).getDriveDistance(),
                        g,//drivetrain.getGyroAngle(),
                        rotation,
                        moduleAngles[i],
                        drivetrain.getSwerveModule(i).getCurrentAngle(),
                        x)
                        );
                    /*encVelLoggers[i].write(String.format("%d,%f\n",
                        iterCount,
                        Math.abs(drivetrain.getSwerveModule(i).getDriveVelocity()))); */
    
                } catch (IOException e) { }
            }
            
        }
        iterCount++;
        
    }

    /**
     * This Command finishes when all 4 of the the robot's wheels stay within TARGET_DISTANCE_BUFFER (inches)
     * of the desired drive distance for DISTANC_CHECK_TIME
     */
    @Override
    public boolean isFinished() {
        boolean inBuffer = true;
        for (int i = 0; i < 4; i++) {
            // check that all wheels are within the TARGET_DISTANCE_BUFFER of the desired travel distance
            inBuffer &= Math.abs(distance - Math.abs(drivetrain.getSwerveModule(i).getDriveDistance())) < TARGET_DISTANCE_BUFFER;
        }

        if (inBuffer) {
            if (!isTimerStarted) {
                finishTimer.start();
                isTimerStarted = true;
            }
        } else {
            finishTimer.stop();
            finishTimer.reset();
            isTimerStarted = false;
        }

        return finishTimer.hasPeriodPassed(DISTANCE_CHECK_TIME);
    }

    @Override
    public void end( boolean isInterrupted) {
        drivetrain.holonomicDrive(0, 0, 0);

        // angleErrorController.disable();
        if (AUTO.LOG)
        {
            for (int i = 0; i < 4; i++) {
                try {
                    if (encPosLoggers[i] != null)
                        encPosLoggers[i].close();
                    if (encVelLoggers[i] != null)
                        encVelLoggers[i].close();
                } catch (IOException e) { }
                encPosLoggers[i] = null;
                encVelLoggers[i] = null;
            }
        }
    }

    public void resetPID() {
        angleErrorController.reset();
        angleErrorController.disableContinuousInput();
        angleErrorController.setSetpoint(0);
        angleErrorController.setIntegratorRange(-1.0, 1.0);
        angleErrorController.setTolerance(0.05, Double.POSITIVE_INFINITY);
        // initialDrivetrainAngle = 0;

        rotationMinOutput = -1;
        forwardMaxOutput = 1;
        strafeMaxOutput = 1;
        rotationMaxOutput = 1;
    }
}