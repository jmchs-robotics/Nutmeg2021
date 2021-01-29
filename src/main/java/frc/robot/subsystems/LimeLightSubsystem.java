package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase{
  // Data from the LimeLight
  /** this is a description of xEntry */
  private NetworkTableEntry TargetValid;
  private NetworkTableEntry HorizontalOffset;
  private NetworkTableEntry VerticalOffset;
  private NetworkTableEntry TargetArea;
  private NetworkTableEntry TargetSkew;
  private NetworkTableEntry Latency;
  private NetworkTableEntry ShortSide; //Sidelength of shortest side of the fitted bounding box (pixels)
  private NetworkTableEntry LongSide; //See above, this one is for the largest side.
  private NetworkTableEntry HorizontalLength; //Horizontal sidelength of the rough bounding box (0 - 320 pixels)
  private NetworkTableEntry VerticalLength; //See above
  private NetworkTableEntry GetPipeIdx; //True active pipeline index of the camera (0 .. 9)
  private NetworkTableEntry CamTran; //Results of a 3D position solution, 6 numbers: Translation (x,y,y) Rotation(pitch,yaw,roll)
  
  // Settable data (everything above "corners")
  private NetworkTableEntry LedMode;
  private NetworkTableEntry camMode;
  private NetworkTableEntry pipeline; //Sets limelight’s current pipeline
  private NetworkTableEntry stream; //Sets limelight’s streaming mode
  private NetworkTableEntry snapshot; 

  // Constructor
  // Remember that constructors have no return type, and their name is the
  // same as the class's
  public LimeLightSubsystem() {
    NetworkTable LimeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
  
    TargetValid      = LimeLightTable.getEntry("tv");
    HorizontalOffset = LimeLightTable.getEntry("tx");
    VerticalOffset   = LimeLightTable.getEntry("ty");
    TargetArea       = LimeLightTable.getEntry("ta");
    TargetSkew       = LimeLightTable.getEntry("ts");
    Latency          = LimeLightTable.getEntry("tl");
    ShortSide        = LimeLightTable.getEntry("tshort");
    LongSide         = LimeLightTable.getEntry("tlong");
    HorizontalLength = LimeLightTable.getEntry("thor");
    VerticalLength   = LimeLightTable.getEntry("tvert");
    GetPipeIdx       = LimeLightTable.getEntry("getpipe");
    CamTran          = LimeLightTable.getEntry("camtran");
  
    LedMode 
    camMode
    pipeline
    stream
    snapshot
  
  }

  
  // Periodic
  @Override
  public void periodic() {
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }


  /**
   * What are some useful things we need to get from the limelight?
   * Hint: we need to set & get most of the entries.
   */
  
}
