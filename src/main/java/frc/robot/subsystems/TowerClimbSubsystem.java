package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TowerClimbSubsystem extends SubsystemBase {

    private final TalonFX towerClimbLead = new TalonFX(Constants.TowerConstants.LEADERCLIMBID);
    // private final TalonFX towerClimbFollow = new TalonFX(Constants.TowerConstants.FOLLOWERCLIMBID);

    private final MotionMagicVoltage towerClimbMotionMagic = new MotionMagicVoltage(0);

    public TowerClimbSubsystem() {

        towerClimbLead.getConfigurator().apply(new TalonFXConfiguration());
        // towerClimbFollow.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration towerClimbConfig = new TalonFXConfiguration();

        towerClimbConfig.Slot0.kS = Constants.TowerConstants.kCLIMB_S;
        towerClimbConfig.Slot0.kV = Constants.TowerConstants.kCLIMB_V;
        towerClimbConfig.Slot0.kA = Constants.TowerConstants.kCLIMB_A;
        towerClimbConfig.Slot0.kP = Constants.TowerConstants.kCLIMB_P;
        towerClimbConfig.Slot0.kI = Constants.TowerConstants.kCLIMB_I;
        towerClimbConfig.Slot0.kD = Constants.TowerConstants.kCLIMB_D;

        towerClimbConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TowerConstants.MM_CRUISE_VELOCITY;
        towerClimbConfig.MotionMagic.MotionMagicAcceleration    = Constants.TowerConstants.MM_ACCELERATION;

        towerClimbConfig.Voltage.withPeakForwardVoltage(12)
                                .withPeakReverseVoltage(12);

        // Brake so the climber holds position when the motor is not commanded
        towerClimbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Soft limits prevent the motor from going past physical bounds
        towerClimbConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        towerClimbConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold  = Constants.TowerConstants.CLIMBPOS;
        towerClimbConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable     = true;
        towerClimbConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold  = Constants.TowerConstants.RESETPOS;

        StatusCode statusOne = StatusCode.StatusCodeNotInitialized;
        // StatusCode statusTwo = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            statusOne = towerClimbLead.getConfigurator().apply(towerClimbConfig);
            // statusTwo = towerClimbFollow.getConfigurator().apply(towerClimbConfig);
            if (statusOne.isOK()) break;
        }
        if (!statusOne.isOK()) {
            System.out.println("Could not apply configs to DeepClimb motor ONE, error code: " + statusOne.toString());
        }
        // if (!statusTwo.isOK()) {
        //   System.out.println("Could not apply configs to DeepClimb motor TWO, error code: " + statusTwo.toString());
        // }

        /* Make sure we start at 0 */
        towerClimbLead.setPosition(0);
        // towerClimbFollow.setPosition(0);
    }

    @Override
    public void periodic() {
        // Watch this on the dashboard to verify encoder direction before tuning.
        // Positive should increase when the climber moves toward CLIMBPOS.
        // If it goes negative, add: towerClimbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        SmartDashboard.putNumber("Climber/Position (rot)", towerClimbLead.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber/Velocity (rot-s)", towerClimbLead.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Climber/Output Voltage (V)", towerClimbLead.getMotorVoltage().getValueAsDouble());
    }

    public void setTowerClimbPosition(double rotations) {
        towerClimbLead.setControl(towerClimbMotionMagic.withPosition(rotations));
        // towerClimbFollow.setControl(towerClimbMotionMagic.withPosition(rotations * -1.0));
    }

    public double displayEncoder() {
        return towerClimbLead.getPosition().getValueAsDouble();
    }
}
