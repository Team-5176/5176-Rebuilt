package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransportSubsystem extends SubsystemBase {
    
    SparkFlex transport = new SparkFlex(12, MotorType.kBrushless);


    public TransportSubsystem()
    {

      SparkFlexConfig transportConfig = new SparkFlexConfig();

      transportConfig.idleMode(IdleMode.kBrake);
      transportConfig.voltageCompensation(12);
      transportConfig.smartCurrentLimit(40);


      transport.configure(transportConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /** This is a method that makes the roller spin */
    public void setTransport(double speed) {
        transport.set(speed);
    }

}