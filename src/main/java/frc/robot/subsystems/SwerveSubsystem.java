package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.SwervePorts;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;



public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    public AHRS gyro;
    public static final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;


    //TODO Kinematics Wİll Be Entered
    static {
        kinematics = new SwerveDriveKinematics(
            new Translation2d(0.3, 0.3), 
            new Translation2d(0.3, -0.3),
            new Translation2d(-0.3, 0.3), 
            new Translation2d(-0.3, -0.3) 
        );
    }



    public SwerveSubsystem() {

        gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);



        frontLeft = new SwerveModule(
            SwervePorts.FRONT_LEFT_DRIVE_MOTOR, SwervePorts.FRONT_LEFT_STEER_MOTOR,
            SwervePorts.FRONT_LEFT_DRIVE_ENCODER_A, SwervePorts.FRONT_LEFT_DRIVE_ENCODER_B,
            SwervePorts.FRONT_LEFT_STEER_ENCODER_A, SwervePorts.FRONT_LEFT_STEER_ENCODER_B,
            Swerve.FRONT_LEFT_MODULE_DRIVE_OFFSET, Swerve.FRONT_LEFT_MODULE_STEER_OFFSET

        );



        frontRight = new SwerveModule(
            SwervePorts.FRONT_RIGHT_DRIVE_MOTOR, SwervePorts.FRONT_RIGHT_STEER_MOTOR,
            SwervePorts.FRONT_RIGHT_DRIVE_ENCODER_A, SwervePorts.FRONT_RIGHT_DRIVE_ENCODER_B,
            SwervePorts.FRONT_RIGHT_STEER_ENCODER_A, SwervePorts.FRONT_RIGHT_STEER_ENCODER_B,
            Swerve.FRONT_RIGHT_MODULE_DRIVE_OFFSET, Swerve.FRONT_RIGHT_MODULE_STEER_OFFSET

        );



        backLeft = new SwerveModule(
            SwervePorts.BACK_LEFT_DRIVE_MOTOR, SwervePorts.BACK_LEFT_STEER_MOTOR,
            SwervePorts.BACK_LEFT_DRIVE_ENCODER_A, SwervePorts.BACK_LEFT_DRIVE_ENCODER_B,
            SwervePorts.BACK_LEFT_STEER_ENCODER_A, SwervePorts.BACK_LEFT_STEER_ENCODER_B,
            Swerve.BACK_LEFT_MODULE_DRIVE_OFFSET, Swerve.BACK_LEFT_MODULE_STEER_OFFSET

        );



        backRight = new SwerveModule(
            SwervePorts.BACK_RIGHT_DRIVE_MOTOR, SwervePorts.BACK_RIGHT_STEER_MOTOR,
            SwervePorts.BACK_RIGHT_DRIVE_ENCODER_A, SwervePorts.BACK_RIGHT_DRIVE_ENCODER_B,
            SwervePorts.BACK_RIGHT_STEER_ENCODER_A, SwervePorts.BACK_RIGHT_STEER_ENCODER_B,
            Swerve.BACK_RIGHT_MODULE_DRIVE_OFFSET, Swerve.BACK_RIGHT_MODULE_STEER_OFFSET
        );



        odometry = new SwerveDriveOdometry(
            kinematics, 
            Rotation2d.fromDegrees(gyro.getYaw()), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );

        //TODO Pathplanner Update
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();

        } catch (Exception e) {
            e.printStackTrace();
            config = null;
        }

        AutoBuilder.configure(

            this::getPose, 
            this::resetOdometry, 
            this::getRobotRelativeSpeeds, 
            (speeds, feedforwards) -> driveRobotRelative(speeds), 
            new PPHolonomicDriveController(
                    Swerve.TRANSLATION_PID, 
                    Swerve.ROTATION_PID 
            ),
            config, 

            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
            );

    }

    //TODO Not Sure

    private void driveRobotRelative(ChassisSpeeds speeds) {
        // Calculate the desired states for each swerve module based on the provided chassis speeds
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
        // Normalize wheel speeds to ensure no module exceeds the maximum speed (e.g., 3.0 m/s)
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 3.0);
    

        // Apply the calculated states to each swerve module

        frontLeft.setDesiredState(applySecondOrderKinematics(swerveModuleStates[0]));
        frontRight.setDesiredState(applySecondOrderKinematics(swerveModuleStates[1]));
        backLeft.setDesiredState(applySecondOrderKinematics(swerveModuleStates[2]));
        backRight.setDesiredState(applySecondOrderKinematics(swerveModuleStates[3]));

        // Send debug information to SmartDashboard for real-time monitoring

        SmartDashboard.putString("FL Desired State", swerveModuleStates[0].toString());
        SmartDashboard.putString("FR Desired State", swerveModuleStates[1].toString());
        SmartDashboard.putString("BL Desired State", swerveModuleStates[2].toString());
        SmartDashboard.putString("BR Desired State", swerveModuleStates[3].toString());

    }

    

    private ChassisSpeeds getRobotRelativeSpeeds() {

        // Obtain the current desired states of each module
        SwerveModuleState frontLeftState = frontLeft.getState();
        SwerveModuleState frontRightState = frontRight.getState();
        SwerveModuleState backLeftState = backLeft.getState();
        SwerveModuleState backRightState = backRight.getState();

    

        // Use the inverse kinematics to compute the robot-relative speeds
        return kinematics.toChassisSpeeds(
            frontLeftState,
            frontRightState,
            backLeftState,
            backRightState
        );

    }

    

    public Command drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        return run(() -> {

            double currentYaw = -gyro.getYaw();

            SmartDashboard.putNumber("Drive/X Speed", xSpeed);
            SmartDashboard.putNumber("Drive/Y Speed", ySpeed);
            SmartDashboard.putNumber("Drive/Rotation Speed", rot);
            SmartDashboard.putBoolean("Drive/Field Relative", fieldRelative);
            SmartDashboard.putNumber("Drive/Gyro Yaw", currentYaw);


            ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeed, ySpeed, rot, Rotation2d.fromDegrees(currentYaw))
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

            var swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 3.0);

            frontLeft.setDesiredState(applySecondOrderKinematics(swerveModuleStates[0]));
            frontRight.setDesiredState(applySecondOrderKinematics(swerveModuleStates[1]));
            backLeft.setDesiredState(applySecondOrderKinematics(swerveModuleStates[2]));
            backRight.setDesiredState(applySecondOrderKinematics(swerveModuleStates[3]));


            SmartDashboard.putString("FL Desired State", swerveModuleStates[0].toString());
            SmartDashboard.putString("FR Desired State", swerveModuleStates[1].toString());
            SmartDashboard.putString("BL Desired State", swerveModuleStates[2].toString());
            SmartDashboard.putString("BR Desired State", swerveModuleStates[3].toString());
        });

    }

//WORK OF ART

    private SwerveModuleState applySecondOrderKinematics(SwerveModuleState state) {
        double vmx = state.speedMetersPerSecond * Math.cos(state.angle.getRadians());
        double vmy = state.speedMetersPerSecond * Math.sin(state.angle.getRadians());
        //double amx = -Math.pow(state.speedMetersPerSecond, 2) * state.angle.getCos();
        //double amy = -Math.pow(state.speedMetersPerSecond, 2) * state.angle.getSin();

        double correctedSpeed = Math.sqrt(vmx * vmx + vmy * vmy);
        double correctedAngle = Math.atan2(vmy, vmx);

        return new SwerveModuleState(correctedSpeed, new Rotation2d(correctedAngle));
    }

    @Override
    public void periodic() {
        double currentYaw = -gyro.getYaw();

        odometry.update(
            Rotation2d.fromDegrees(currentYaw),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );



        SmartDashboard.putString("Odometry Pose", odometry.getPoseMeters().toString());
        SmartDashboard.putNumber("Gyro Yaw", currentYaw);
        SmartDashboard.putString("FL Position", frontLeft.getPosition().toString());
        SmartDashboard.putString("FR Position", frontRight.getPosition().toString());
        SmartDashboard.putString("BL Position", backLeft.getPosition().toString());
        SmartDashboard.putString("BR Position", backRight.getPosition().toString());
    }



    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }



    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(-gyro.getYaw()), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose
        );

    }

}



class SwerveModule {
    private final PWMSparkMax driveMotor;
    private final PWMSparkMax steerMotor;
    private final Encoder driveEncoder;
    private final Encoder steerEncoder;
    private final double driveEncoderOffset;
    private final double steerEncoderOffset;


    public SwerveModule(int driveMotorPort, int steerMotorPort, int driveEncoderPortA, int driveEncoderPortB, int steerEncoderPortA, int steerEncoderPortB, double driveEncoderOffset, double steerEncoderOffset) {
        driveMotor = new PWMSparkMax(driveMotorPort);
        steerMotor = new PWMSparkMax(steerMotorPort);
        driveEncoder = new Encoder(driveEncoderPortA, driveEncoderPortB);
        steerEncoder = new Encoder(steerEncoderPortA, steerEncoderPortB);

        this.driveEncoderOffset = driveEncoderOffset;
        this.steerEncoderOffset = steerEncoderOffset;
    }



    public SwerveModuleState getState() {
        double adjustedDriveRate = driveEncoder.getRate() + driveEncoderOffset;
        double adjustedSteerDistance = steerEncoder.getDistance() + steerEncoderOffset;

        return new SwerveModuleState(
            adjustedDriveRate,
            new Rotation2d(Math.toRadians(adjustedSteerDistance))
        );

    }



    public SwerveModulePosition getPosition() {
        double adjustedDriveDistance = driveEncoder.getDistance() + driveEncoderOffset;
        double adjustedSteerDistance = steerEncoder.getDistance() + steerEncoderOffset;

        return new SwerveModulePosition(
            adjustedDriveDistance,
            new Rotation2d(Math.toRadians(adjustedSteerDistance))
        );

    }



    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentAngle = getState().angle;
        double deltaAngle = desiredState.angle.minus(currentAngle).getRadians();

        if (Math.abs(deltaAngle) > Math.PI / 2) {
            desiredState = new SwerveModuleState(
                -desiredState.speedMetersPerSecond,
                desiredState.angle.rotateBy(Rotation2d.fromDegrees(180))
            );
        }

        driveMotor.set(desiredState.speedMetersPerSecond / 3.0);
        steerMotor.set(desiredState.angle.getRadians() / Math.PI);
    }

} 
