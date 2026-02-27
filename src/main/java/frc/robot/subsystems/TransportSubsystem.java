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

public class TransportSubsystem extends SubsystemBase {
    
    SparkFlex transport = new SparkFlex(Constants.TransportConstants.TRANSPORTID, MotorType.kBrushless);


    public TransportSubsystem()
    {

      SparkFlexConfig transportConfig = new SparkFlexConfig();
      FeedForwardConfig transportFeedForwardConfig = new FeedForwardConfig();

      transportConfig.idleMode(IdleMode.kBrake);
      transportConfig.voltageCompensation(Constants.TransportConstants.Transport_MOTORS_VOLTAGE);
      transportConfig.smartCurrentLimit(Constants.TransportConstants.Transport_MOTORS_CURRENT_LIMIT);

      // transportFeedForwardConfig
      //                     .kS(Constants.TransportConstants.kTransportS)
      //                     .kV(Constants.TransportConstants.kTransportV)
      //                     .kA(Constants.TransportConstants.kTransportA);

      transportConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    Constants.TransportConstants.kTransportP,
                    Constants.TransportConstants.kTransportI,
                    Constants.TransportConstants.kTransportD);
      ClosedLoopConfig transportClosedLoopConfig = transportConfig.closedLoop;
              transportClosedLoopConfig.apply(transportFeedForwardConfig);
              transportConfig.apply(transportClosedLoopConfig);


          transport.configure(transportConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /** This is a method that makes the roller spin */
    public void setTransport(double velocityRPM) {

      transport.getClosedLoopController().setSetpoint(velocityRPM, ControlType.kVelocity);
    }

}