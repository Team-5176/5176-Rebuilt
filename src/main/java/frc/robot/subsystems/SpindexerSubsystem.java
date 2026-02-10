package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase { 

    private final SparkMax spindexer = new SparkMax(Constants.SpindexerConstants.SPINDEXERID, MotorType.kBrushless);

    public SpindexerSubsystem() {

        SparkMaxConfig spindexerConfig  = new SparkMaxConfig();
        FeedForwardConfig spindexerFeedForwardConfig = new FeedForwardConfig();

        spindexerConfig.idleMode(IdleMode.kBrake);
        spindexerConfig.smartCurrentLimit(Constants.SpindexerConstants.SPINDEX_MOTORS_CURRENT_LIMIT);
        spindexerConfig.voltageCompensation(Constants.SpindexerConstants.SPINDEX_MOTORS_VOLTAGE);

        spindexerFeedForwardConfig
                          .kS(Constants.SpindexerConstants.kSpindexS)
                          .kV(Constants.SpindexerConstants.kSpindexV)
                          .kA(Constants.SpindexerConstants.kSpindexA);

        spindexerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    Constants.SpindexerConstants.kSpindexP,
                    Constants.SpindexerConstants.kSpindexI,
                    Constants.SpindexerConstants.kSpindexD);
        ClosedLoopConfig spindexerClosedLoopConfig = spindexerConfig.closedLoop;
              spindexerClosedLoopConfig.apply(spindexerFeedForwardConfig);
              spindexerConfig.apply(spindexerClosedLoopConfig);


        spindexer.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void runSpindexer(double velocityRPM) {
        spindexer.getClosedLoopController().setSetpoint(velocityRPM, ControlType.kVelocity);
    }
}
