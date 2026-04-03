package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveTemplate;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveTemplate frontLeft = new SwerveTemplate(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftSteerMotorPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftSteerEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,            
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad);

    private final SwerveTemplate frontRight = new SwerveTemplate(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightSteerMotorPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightSteerEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad);

    private final SwerveTemplate backLeft = new SwerveTemplate(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftSteerMotorPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftSteerEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad);

    private final SwerveTemplate backRight = new SwerveTemplate(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightSteerMotorPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightSteerEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad);

    private final Pigeon2 gyro = new Pigeon2(DriveConstants.kPigeon2Port);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        new SwerveModulePosition[]{
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
        },
        new Pose2d(0,0, new Rotation2d(0))
        // TODO: change to actual starting pose when doing AUTO stuff
        );

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
          frontLeft.getSwerveModulePosition(),   
          frontRight.getSwerveModulePosition(), 
          backLeft.getSwerveModulePosition(),
          backRight.getSwerveModulePosition()}, 
          pose);;
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
          frontLeft.getSwerveModulePosition(),   
          frontRight.getSwerveModulePosition(), 
          backLeft.getSwerveModulePosition(),
          backRight.getSwerveModulePosition()});
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
