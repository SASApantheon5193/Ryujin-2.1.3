package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class CoralSubsystem {
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();
    public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the arm motor
      armConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(0.25);

      // Configure basic settings of the elevator motor
      elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
      elevatorConfig
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);

      // Configure basic settings of the intake motor
      intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    }
  }

  public static final class AlgaeSubsystem {
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final SparkFlexConfig armConfig = new SparkFlexConfig();

    static {
      // Configure basic setting of the arm motor
      armConfig.smartCurrentLimit(40);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed
          // loop slot, as it will default to slot 0.
          .p(0.1)
          .outputRange(-0.5, 0.5);

      // Configure basic settings of the intake motor
      intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    }
  }
  
public static RobotConfig getDefault() throws ParseException {
    try {
        return RobotConfig.fromGUISettings();
    } catch (IOException e) {
        // Log the error if needed
        System.err.println("Failed to load RobotConfig from GUI settings, using fallback.");
        e.printStackTrace();

        // Fallback hardcoded config - customize if necessary
        return new RobotConfig(
            50.0, // massKG
            5.0,  // MOI
            new ModuleConfig(
                0.0702,  // wheelRadiusMeters
                4.8,     // maxDriveVelocityMPS
                1.19,    // wheelCOF
                edu.wpi.first.math.system.plant.DCMotor.getNEO(1).withReduction(6.75), // Example
                50,      // driveCurrentLimit
                1        // numMotors
            ),
            new edu.wpi.first.math.geometry.Translation2d[] {
                new edu.wpi.first.math.geometry.Translation2d(0.27305, 0.27305),  // FL
                new edu.wpi.first.math.geometry.Translation2d(0.27305, -0.27305), // FR
                new edu.wpi.first.math.geometry.Translation2d(-0.27305, 0.27305), // BL
                new edu.wpi.first.math.geometry.Translation2d(-0.27305, -0.27305) // BR
            }
        );
    }
}

  
}