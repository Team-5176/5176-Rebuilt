package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    
    private final SparkFlex shooterLeader = new SparkFlex(Constants.ShooterConstants.LEADERSHOOTERID, MotorType.kBrushless);
    private final SparkFlex shooterFollower = new SparkFlex(Constants.ShooterConstants.FOLLOWERSHOOTERID, MotorType.kBrushless);


    public ShooterSubsystem()
    {
      SparkFlexConfig shooterLeaderConfig = new SparkFlexConfig();
      SparkFlexConfig shooterFollowerConfig = new SparkFlexConfig();

      FeedForwardConfig shooterFeedForwardConfig = new FeedForwardConfig();

      shooterFeedForwardConfig
            .kS(Constants.ShooterConstants.kS) // Needed to overcome static friction
            .kV(Constants.ShooterConstants.kV) // Velocity gain?
            .kA(Constants.ShooterConstants.kA);// Acceleration gain?

      shooterLeaderConfig.idleMode(IdleMode.kBrake);
      shooterLeaderConfig.voltageCompensation(Constants.ShooterConstants.SHOOTER_MOTORS_VOLTAGE);
      shooterLeaderConfig.smartCurrentLimit(Constants.ShooterConstants.SHOOTER_MOTORS_CURRENT_LIMIT);

      shooterLeaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.ShooterConstants.kP, Constants.ShooterConstants.kI, Constants.ShooterConstants.kD)
            // .iZone(Constants.ShooterConstants.kIz)
            .outputRange(-1, 1);


      ClosedLoopConfig shooterClosedLoopConfig = shooterLeaderConfig.closedLoop;
      shooterClosedLoopConfig.apply(shooterFeedForwardConfig);
      shooterLeaderConfig.apply(shooterClosedLoopConfig);

      // These lines NEED to be after defining config parameters for the lead motor
      shooterFollowerConfig.apply(shooterLeaderConfig);
      shooterFollowerConfig.follow(12, true);

      shooterLeader.configure(shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      shooterFollower.configure(shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /* This is a method that makes the roller spin at a target velocity */
    public void setShooterVelocity(double velocityRPM) {
        shooterLeader.getClosedLoopController().setSetpoint(velocityRPM, ControlType.kVelocity);
    }
    
    public boolean isShooting() {

        return Math.abs(shooterLeader.getEncoder().getVelocity()) > 2000; // Check if velocity > 100 RPM
    }
    
    /** Check if shooter is at target speed (within tolerance) */
    public boolean atTargetSpeed() {
        double currentVelocity = shooterLeader.getEncoder().getVelocity();
        return Math.abs(currentVelocity - Constants.ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM) < 100; // Within 100 RPM
    }
    
    /** Get current velocity in RPM */
    public double getVelocityRPM() {
        return shooterLeader.getEncoder().getVelocity();
    }
}
