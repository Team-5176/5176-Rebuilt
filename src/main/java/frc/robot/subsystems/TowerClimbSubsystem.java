package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TowerClimbSubsystem extends SubsystemBase {

    private final TalonFX towerClimbLead = new TalonFX(Constants.TowerConstants.LEADERCLIMBID);
    private final TalonFX towerClimbFollow = new TalonFX(Constants.TowerConstants.FOLLOWERCLIMBID);

    private final SparkMax towerClimbFlippers = new SparkMax(0,MotorType.kBrushless);

    private final PositionVoltage towerClimbPositionVoltage = new PositionVoltage(0);

    public TowerClimbSubsystem() {

        SparkMaxConfig flippersConfig  = new SparkMaxConfig();
        FeedForwardConfig flippersFeedForwardConfig = new FeedForwardConfig();

        flippersConfig.idleMode(IdleMode.kBrake);
        flippersConfig.smartCurrentLimit(Constants.TowerConstants.FLIPPERS_MOTOR_CURRENT_LIMIT);
        flippersConfig.voltageCompensation(Constants.TowerConstants.FLIPPERS_MOTOR_VOLTAGE);

        flippersFeedForwardConfig
                          .kS(Constants.TowerConstants.kFlippersS)
                          .kV(Constants.TowerConstants.kFlippersV)
                          .kA(Constants.TowerConstants.kFlippersA);

        flippersConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    Constants.TowerConstants.kFlippersP,
                    Constants.TowerConstants.kFlippersI,
                    Constants.TowerConstants.kFlippersD);
        ClosedLoopConfig spindexerClosedLoopConfig = flippersConfig.closedLoop;
              spindexerClosedLoopConfig.apply(flippersFeedForwardConfig);
              flippersConfig.apply(spindexerClosedLoopConfig);


        towerClimbFlippers.configure(flippersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        towerClimbLead.getConfigurator().apply(new TalonFXConfiguration());
        towerClimbFollow.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration towerClimbConfig = new TalonFXConfiguration();

        towerClimbConfig.Slot0.kP = Constants.TowerConstants.kCLIMB_P;
        towerClimbConfig.Slot0.kI = Constants.TowerConstants.kCLIMB_I;
        towerClimbConfig.Slot0.kD = Constants.TowerConstants.kCLIMB_D;

        towerClimbConfig.Voltage.withPeakForwardVoltage(12)
                                .withPeakReverseVoltage(12);
        towerClimbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        StatusCode statusOne = StatusCode.StatusCodeNotInitialized;
        StatusCode statusTwo = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
        statusOne = towerClimbLead.getConfigurator().apply(towerClimbConfig);
        statusTwo = towerClimbFollow.getConfigurator().apply(towerClimbConfig);
        if (statusOne.isOK() && statusTwo.isOK()) break;
        }
        if (!statusOne.isOK()) {
            System.out.println("Could not apply configs to DeepClimb motor ONE, error code: " + statusOne.toString());
        }
        if (!statusTwo.isOK()) {
            System.out.println("Could not apply configs to DeepClimb motor TWO, error code: " + statusTwo.toString());
        }

        /* Make sure we start at 0 */
        towerClimbLead.setPosition(0);
        towerClimbFollow.setPosition(0);
    }

    public void setTowerClimbPosition(double rotations) {

        double rawRotations = rotations*25; 
        towerClimbLead.setControl(towerClimbPositionVoltage.withPosition(rawRotations));
        towerClimbFollow.setControl(towerClimbPositionVoltage.withPosition(rawRotations * -1.0));
    }

    public void setFlippersPosition(double rotations) {

        towerClimbFlippers.getClosedLoopController().setSetpoint(rotations, ControlType.kPosition);
    }

    
}
