package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    SparkFlex shooterLeader = new SparkFlex(12, MotorType.kBrushless);
    SparkFlex shooterFollower = new SparkFlex(11, MotorType.kBrushless);


    public ShooterSubsystem()
    {

      shooterLeader.setCANTimeout(250);

      SparkFlexConfig shooterLeaderConfig = new SparkFlexConfig();
      SparkFlexConfig shooterFollowerConfig = new SparkFlexConfig();

      shooterLeaderConfig.idleMode(IdleMode.kBrake);
      shooterLeaderConfig.voltageCompensation(12);
      shooterLeaderConfig.smartCurrentLimit(40);

      shooterFollowerConfig.apply(shooterLeaderConfig);
      shooterFollowerConfig.follow(12, true);


      shooterLeader.configure(shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      shooterFollower.configure(shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /** This is a method that makes the roller spin */
    public void runRoller(double speed) {
      shooterLeader.set(speed);
    }
    
    public boolean isShooting() {
        return shooterLeader.get() > 0;
    }

}
