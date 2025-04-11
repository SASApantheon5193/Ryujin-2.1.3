package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.DriveConstants;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.subsystems.VisionHelper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;


import java.io.IOException;
import org.json.simple.parser.ParseException;

public class DriveSubsystem extends SubsystemBase {
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset);
  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset);
  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset);
  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset);

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final Field2d m_field = new Field2d();
  private final VisionHelper visionHelper;
  private final RobotConfig config;

  private AprilTagFieldLayout fieldLayout;
  private final Field2d visionField = new Field2d();



  public DriveSubsystem() {
    RobotConfig config;
    try {
        config = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
        try {
            config = Configs.getDefault();
        } catch (ParseException ex) {
            ex.printStackTrace();
            throw new RuntimeException("Failed to load RobotConfig from GUI and fallback default!", ex);
        }
        e.printStackTrace();
    }

    this.config = config;  // Corrected line!

    // Load 2025 Reefscape Field Layout
try {
  fieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
  SmartDashboard.putString("Vision/FieldLayout", "2025 Reefscape Loaded");
  SmartDashboard.putNumber("Vision/FieldLayout Tag Count", fieldLayout.getTags().size());
} catch (Exception e) {
  DriverStation.reportError("Failed to load Reefscape AprilTag layout: " + e.getMessage(), e.getStackTrace());
  SmartDashboard.putString("Vision/FieldLayout", "Failed to Load");
}


    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroRotation(),
        getModulePositions(),
        new Pose2d()
    );

    visionHelper = new VisionHelper(m_poseEstimator, m_gyro);

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new PPHolonomicDriveController(
            new PIDConstants(3.16, 0.0, 0.7),
            new PIDConstants(4.9, 0.0, 0.59)
        ),
        config,  // correct usage
        () -> DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        this
    );
}


  @Override
  public void periodic() {
    visionHelper.update();

    m_poseEstimator.update(getGyroRotation(), getModulePositions());
    m_field.setRobotPose(getPose());
    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(getPose());
visionField.setRobotPose(visionHelper.getLastVisionPose());  // You'll need to add this method in VisionHelper
SmartDashboard.putData("Vision Field", visionField);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
  }

  public Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ));
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    }
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public Command zeroHeadingCommand() {
    return runOnce(() -> m_gyro.reset());
  }

  public Command resetIMUAndOdometryCommand(Pose2d startingPose) {
    return runOnce(() -> {
      m_gyro.reset();
      resetOdometry(startingPose);
    });
  }
  /** Sets the wheels into an X formation to prevent movement. */
public Command setXCommand() {
  return run(() -> {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  });
}

}
