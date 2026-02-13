package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase  {

    
    private final SparkFlex intakeRoller = new SparkFlex(Constants.IntakeConstants.ROLLERID, MotorType.kBrushless);
    private final SparkMax intakeArm = new SparkMax(Constants.IntakeConstants.ARMID, MotorType.kBrushless);

    public IntakeSubsystem(){ 

        SparkFlexConfig intakeRollerConfig = new SparkFlexConfig();
        FeedForwardConfig intakeRollerFeedForwardConfig = new FeedForwardConfig();

        SparkMaxConfig intakeArmConfig = new SparkMaxConfig();
        FeedForwardConfig intakeArmFeedForwardConfig = new FeedForwardConfig();

        intakeRollerConfig.idleMode(IdleMode.kCoast);
        intakeRollerConfig.smartCurrentLimit(Constants.IntakeConstants.INTAKE_ROLLER_MOTORS_CURRENT_LIMIT);
        intakeRollerConfig.voltageCompensation(Constants.IntakeConstants.INTAKE_ROLLER_MOTORS_VOLTAGE);

        intakeRollerFeedForwardConfig
                            .kS(Constants.IntakeConstants.kRollerS)
                            .kV(Constants.IntakeConstants.kRollerV)
                            .kA(Constants.IntakeConstants.kRollerA);

        intakeRollerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    Constants.IntakeConstants.kRollerP,
                    Constants.IntakeConstants.kRollerI,
                    Constants.IntakeConstants.kRollerD);
        ClosedLoopConfig intakeRollerClosedLoopConfig = intakeRollerConfig.closedLoop;
        intakeRollerClosedLoopConfig.apply(intakeRollerFeedForwardConfig);
        intakeRollerConfig.apply(intakeRollerClosedLoopConfig);

        intakeArmConfig.idleMode(IdleMode.kBrake);
        intakeArmConfig.smartCurrentLimit(Constants.IntakeConstants.INTAKE_ARM_MOTORS_CURRENT_LIMIT);
        intakeArmConfig.voltageCompensation(Constants.IntakeConstants.INTAKE_ARM_MOTORS_CURRENT_LIMIT);

        intakeArmFeedForwardConfig
                            .kS(Constants.IntakeConstants.kArmS)
                            .kV(Constants.IntakeConstants.kArmV)
                            .kA(Constants.IntakeConstants.kArmA);

        intakeArmConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                    Constants.IntakeConstants.kArmP,
                    Constants.IntakeConstants.kArmI,
                    Constants.IntakeConstants.kArmD);
        ClosedLoopConfig intakeArmClosedLoopConfig = intakeArmConfig.closedLoop;
        intakeArmClosedLoopConfig.apply(intakeArmFeedForwardConfig);
        intakeArmConfig.apply(intakeArmClosedLoopConfig);

        

        intakeRoller.configure(intakeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeArm.configure(intakeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }  

    public void spinIntake(double velocityRPM) {

        intakeRoller.getClosedLoopController().setSetpoint(velocityRPM, ControlType.kVelocity);
    }

    public void deployIntake(double deployPoint) {

        intakeArm.getClosedLoopController().setSetpoint(deployPoint, ControlType.kPosition);
    }

    public double getVelocity()
    {
        return intakeRoller.getEncoder().getVelocity();
    }

    public boolean isIntaking()
    {
        return  Math.abs(intakeRoller.getEncoder().getVelocity()) > 2000;
    }
}