package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.Supplier;

/**
 * Drives a YAGSL swerve to a field-relative target pose using PID on X, Y, and heading.
 */
public class DriveToPoseCommand extends Command
{
  private static final String TRANSLATION_KP_KEY = "DriveToPose/Translation kP";
  private static final String TRANSLATION_KD_KEY = "DriveToPose/Translation kD";
  private static final String ROTATION_KP_KEY    = "DriveToPose/Rotation kP";
  private static final String ROTATION_KD_KEY    = "DriveToPose/Rotation kD";
  private static final String XY_TOLERANCE_KEY   = "DriveToPose/XY Tolerance M";
  private static final String THETA_TOLERANCE_KEY = "DriveToPose/Theta Tolerance Deg";
  private static final String TIMEOUT_KEY        = "DriveToPose/Timeout Sec";

  private final SwerveSubsystem drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  private final Timer            timer = new Timer();

  private final Transform2d targetTolerance;
  private final double      defaultTimeoutSeconds;

  private final PIDController xController = new PIDController(
      Constants.DriveToPIDConstants.TRANSLATION_KP,
      0,
      Constants.DriveToPIDConstants.TRANSLATION_KD);
  private final PIDController yController = new PIDController(
      Constants.DriveToPIDConstants.TRANSLATION_KP,
      0,
      Constants.DriveToPIDConstants.TRANSLATION_KD);
  private final PIDController thetaController = new PIDController(
      Constants.DriveToPIDConstants.ROTATION_KP,
      0,
      Constants.DriveToPIDConstants.ROTATION_KD);

  private Pose2d targetPose = new Pose2d();

  public DriveToPoseCommand(SwerveSubsystem drivetrain, Supplier<Pose2d> poseSupplier)
  {
    this(
        drivetrain,
        poseSupplier,
        new Transform2d(
            Constants.DriveToPIDConstants.TRANSLATION_TOLERANCE_M,
            Constants.DriveToPIDConstants.TRANSLATION_TOLERANCE_M,
            Rotation2d.fromRadians(Constants.DriveToPIDConstants.ROTATION_TOLERANCE_RAD)),
        3.0);
  }

  public DriveToPoseCommand(
      SwerveSubsystem drivetrain,
      Supplier<Pose2d> poseSupplier,
      Transform2d tolerance,
      double timeoutSeconds)
  {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.targetTolerance = tolerance;
    this.defaultTimeoutSeconds = timeoutSeconds;

    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    publishDefaultTunables();
    refreshTunables();
  }

  @Override
  public void initialize()
  {
    targetPose = poseSupplier.get();

    refreshTunables();
    xController.reset();
    yController.reset();
    thetaController.reset();
    timer.restart();

    SmartDashboard.putString("DriveToPose/Target Pose",
        String.format("(%.2f, %.2f, %.1f deg)",
            targetPose.getX(),
            targetPose.getY(),
            targetPose.getRotation().getDegrees()));
  }

  @Override
  public void execute()
  {
    refreshTunables();
    Pose2d currentPose = drivetrain.getPose();

    double xVelocity = clamp(
        xController.calculate(currentPose.getX(), targetPose.getX()),
        drivetrain.getSwerveDrive().getMaximumChassisVelocity());
    double yVelocity = clamp(
        yController.calculate(currentPose.getY(), targetPose.getY()),
        drivetrain.getSwerveDrive().getMaximumChassisVelocity());
    double thetaVelocity = clamp(
        thetaController.calculate(
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians()),
        drivetrain.getSwerveDrive().getMaximumChassisAngularVelocity());

    drivetrain.driveFieldOriented(new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity));

    SmartDashboard.putNumber("DriveToPose/X Error M", targetPose.getX() - currentPose.getX());
    SmartDashboard.putNumber("DriveToPose/Y Error M", targetPose.getY() - currentPose.getY());
    SmartDashboard.putNumber(
        "DriveToPose/Theta Error Deg",
        targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());
  }

  @Override
  public boolean isFinished()
  {
    Transform2d error = new Transform2d(drivetrain.getPose(), targetPose);
    return (Math.abs(error.getX()) <= targetTolerance.getX()
            && Math.abs(error.getY()) <= targetTolerance.getY()
            && Math.abs(error.getRotation().getRadians()) <= targetTolerance.getRotation().getRadians())
           || timer.hasElapsed(getTimeoutSeconds());
  }

  @Override
  public void end(boolean interrupted)
  {
    drivetrain.drive(new ChassisSpeeds());
  }

  private static double clamp(double value, double maxMagnitude)
  {
    return Math.max(-maxMagnitude, Math.min(maxMagnitude, value));
  }

  private void publishDefaultTunables()
  {
    putNumberIfMissing(TRANSLATION_KP_KEY, Constants.DriveToPIDConstants.TRANSLATION_KP);
    putNumberIfMissing(TRANSLATION_KD_KEY, Constants.DriveToPIDConstants.TRANSLATION_KD);
    putNumberIfMissing(ROTATION_KP_KEY, Constants.DriveToPIDConstants.ROTATION_KP);
    putNumberIfMissing(ROTATION_KD_KEY, Constants.DriveToPIDConstants.ROTATION_KD);
    putNumberIfMissing(XY_TOLERANCE_KEY, targetTolerance.getX());
    putNumberIfMissing(THETA_TOLERANCE_KEY, targetTolerance.getRotation().getDegrees());
    putNumberIfMissing(TIMEOUT_KEY, defaultTimeoutSeconds);
  }

  private void refreshTunables()
  {
    xController.setPID(
        SmartDashboard.getNumber(TRANSLATION_KP_KEY, Constants.DriveToPIDConstants.TRANSLATION_KP),
        0,
        SmartDashboard.getNumber(TRANSLATION_KD_KEY, Constants.DriveToPIDConstants.TRANSLATION_KD));
    yController.setPID(
        SmartDashboard.getNumber(TRANSLATION_KP_KEY, Constants.DriveToPIDConstants.TRANSLATION_KP),
        0,
        SmartDashboard.getNumber(TRANSLATION_KD_KEY, Constants.DriveToPIDConstants.TRANSLATION_KD));
    thetaController.setPID(
        SmartDashboard.getNumber(ROTATION_KP_KEY, Constants.DriveToPIDConstants.ROTATION_KP),
        0,
        SmartDashboard.getNumber(ROTATION_KD_KEY, Constants.DriveToPIDConstants.ROTATION_KD));

    double xyTolerance = SmartDashboard.getNumber(XY_TOLERANCE_KEY, targetTolerance.getX());
    double thetaToleranceRadians = Math.toRadians(
        SmartDashboard.getNumber(THETA_TOLERANCE_KEY, targetTolerance.getRotation().getDegrees()));
    xController.setTolerance(xyTolerance);
    yController.setTolerance(xyTolerance);
    thetaController.setTolerance(thetaToleranceRadians);
  }

  private double getTimeoutSeconds()
  {
    return SmartDashboard.getNumber(TIMEOUT_KEY, defaultTimeoutSeconds);
  }

  private static void putNumberIfMissing(String key, double value)
  {
    if (!SmartDashboard.containsKey(key))
    {
      SmartDashboard.putNumber(key, value);
    }
  }
}
