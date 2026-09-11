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

        spindexerConfig.idleMode(IdleMode.kCoast);
        spindexerConfig.smartCurrentLimit(Constants.SpindexerConstants.SPINDEX_MOTORS_CURRENT_LIMIT);
        spindexerConfig.voltageCompensation(Constants.SpindexerConstants.SPINDEX_MOTORS_VOLTAGE);
        spindexerConfig.encoder.uvwMeasurementPeriod(10);


        spindexerFeedForwardConfig
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
    
    // velocityRPM is real/output-shaft RPM; converted below to motor-shaft RPM for the
    // closed-loop controller, which reads the motor's encoder directly (pre-gear-reduction).
    public void runSpindexer(double velocityRPM) {
        if (velocityRPM == 0) {
            // Let the motor coast to a stop instead of actively PID-holding 0 RPM,
            // which was fighting small perturbations and clicking as it hunted around zero.
            spindexer.stopMotor();
        } else {
            double motorRPM = velocityRPM * Constants.SpindexerConstants.SPINDEXER_GEAR_REDUCTION;
            spindexer.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity);
        }
    }

    // added temporly to test Spindexer
     public double getVelocity()
    {
        // Real/output-shaft RPM
        return spindexer.getEncoder().getVelocity() / Constants.SpindexerConstants.SPINDEXER_GEAR_REDUCTION;
    }

    public boolean isSpindexing()
    {
        return  Math.abs(getVelocity()) > 400; // real-RPM equivalent of the old 2000 motor-RPM threshold
    }

   }
