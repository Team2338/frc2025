package team.gif.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.lib.drivePace;
import team.gif.lib.logging.TelemetryFileLogger;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;
import team.gif.robot.subsystems.drivers.swerve.CANCoderEncoder;
import team.gif.robot.subsystems.drivers.swerve.DriveMotor;
import team.gif.robot.subsystems.drivers.swerve.Encoder;
import team.gif.robot.subsystems.drivers.swerve.SparkMaxDriveMotor;
import team.gif.robot.subsystems.drivers.swerve.SwerveModule;
import team.gif.robot.subsystems.drivers.swerve.TalonSRXTurnMotorEncoder;
import team.gif.robot.subsystems.drivers.swerve.TurnMotor;

/**
 * @author Rohan Cherukuri
 * @since 2/14/22
 */
public class SwerveDrivetrainMk4 extends SubsystemBase {
    public static SwerveModule fL;
    public static SwerveModule fR;
    public static SwerveModule rL;
    public static SwerveModule rR;

    private static DriveMotor fLDriveMotor;
    private static DriveMotor fRDriveMotor;
    private static DriveMotor rLDriveMotor;
    private static DriveMotor rRDriveMotor;

    private static TurnMotor fLTurnMotor;
    private static TurnMotor fRTurnMotor;
    private static TurnMotor rLTurnMotor;
    private static TurnMotor rRTurnMotor;

    private static Encoder fLEncoder;
    private static Encoder fREncoder;
    private static Encoder rLEncoder;
    private static Encoder rREncoder;

    private static SwerveDriveOdometry odometry;
    private static drivePace drivePace;

    // Network Table publishers for the swerve
    // states so that we can use them in advantage scope
    private StructArrayPublisher<SwerveModuleState> targetPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("TargetSwerveState", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> actualPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("ActualSwerveState", SwerveModuleState.struct).publish();



    /**
     * Constructor for swerve drivetrain using 4 swerve modules using NEOs to drive and TalonSRX to control turning
     */
    public SwerveDrivetrainMk4() {
        super();

        fLDriveMotor = new SparkMaxDriveMotor(RobotMap.FRONT_LEFT_DRIVE_MOTOR_ID);
        fLTurnMotor = new TalonSRXTurnMotorEncoder(RobotMap.FRONT_LEFT_TURNING_MOTOR_ID);
        fLEncoder = new CANCoderEncoder(RobotMap.FRONT_LEFT_CANCODER_ID);
        fRDriveMotor = new SparkMaxDriveMotor(RobotMap.FRONT_RIGHT_DRIVE_MOTOR_ID);
        fRTurnMotor = new TalonSRXTurnMotorEncoder(RobotMap.FRONT_RIGHT_TURNING_MOTOR_ID);
        fREncoder = new CANCoderEncoder(RobotMap.FRONT_LEFT_CANCODER_ID);
        rLDriveMotor = new SparkMaxDriveMotor(RobotMap.REAR_LEFT_DRIVE_MOTOR_ID);
        rLTurnMotor = new TalonSRXTurnMotorEncoder(RobotMap.REAR_LEFT_TURNING_MOTOR_ID);
        rLEncoder = new CANCoderEncoder(RobotMap.REAR_LEFT_CANCODER_ID);
        rRDriveMotor = new SparkMaxDriveMotor(RobotMap.REAR_RIGHT_DRIVE_MOTOR_ID);
        rRTurnMotor = new TalonSRXTurnMotorEncoder(RobotMap.REAR_RIGHT_TURNING_MOTOR_ID);
        rREncoder = new CANCoderEncoder(RobotMap.REAR_RIGHT_CANCODER_ID);

        fL = new SwerveModule(
                fLDriveMotor,
                fLTurnMotor,
                fLEncoder,
                true,
                true,
                Constants.DrivetrainMK3.FRONT_LEFT_OFFSET,
                Constants.ModuleConstantsMK3.DrivetrainPID.frontLeftFF,
                Constants.ModuleConstantsMK3.DrivetrainPID.frontLeftP
        );

        fR = new SwerveModule (
                fRDriveMotor,
                fRTurnMotor,
                fREncoder,
                true,
                false,
                Constants.DrivetrainMK3.FRONT_RIGHT_OFFSET,
                Constants.ModuleConstantsMK3.DrivetrainPID.frontRightFF,
                Constants.ModuleConstantsMK3.DrivetrainPID.frontRightP
        );

        rL = new SwerveModule (
                rLDriveMotor,
                rLTurnMotor,
                rLEncoder,
                true,
                true,
                Constants.DrivetrainMK3.REAR_LEFT_OFFSET,
                Constants.ModuleConstantsMK3.DrivetrainPID.rearLeftFF,
                Constants.ModuleConstantsMK3.DrivetrainPID.rearLeftP
        );

        rR = new SwerveModule (
                rRDriveMotor,
                rRTurnMotor,
                rREncoder,
                true,
                false,
                Constants.DrivetrainMK3.REAR_RIGHT_OFFSET,
                Constants.ModuleConstantsMK3.DrivetrainPID.rearRightFF,
                Constants.ModuleConstantsMK3.DrivetrainPID.rearRightP
        );

        odometry = new SwerveDriveOdometry(Constants.DrivetrainMK3.DRIVE_KINEMATICS, Robot.pigeon.getRotation2d(), getPosition(), new Pose2d(0, 0, new Rotation2d(0)));

//        resetHeading();
        resetDriveEncoders();

        drivePace = drivePace.COAST_FR;
        //Autos stuff
        //TODO: put this in constants. need to ref api docs
        RobotConfig ppConfig = null;
        try{
            ppConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeed,
                this::setModuleChassisSpeeds,
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(Constants.DrivetrainAuto.kP_FORWARD, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(Constants.DrivetrainAuto.kP_ROTATION, 0.0, 0.0) // Rotation PID constants
                ),
                ppConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if( alliance.isPresent() ){
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                 },
                this
        );
    }

    public SwerveDrivetrainMk4(TelemetryFileLogger logger) {
        this();

//        logger.addMetric("FL_Rotation", fL::getTurningHeading);
//        logger.addMetric("FR_Rotation", fR::getTurningHeading);
//        logger.addMetric("RL_Rotation", rL::getTurningHeading);
//        logger.addMetric("RR_Rotation", rR::getTurningHeading);
//
//        logger.addMetric("FL_Drive_Command", () -> fL.getDriveMotor().getDutyCycle().getValueAsDouble());
//        logger.addMetric("FR_Drive_Command", () -> fR.getDriveMotor().getDutyCycle().getValueAsDouble());
//        logger.addMetric("RL_Drive_Command", () -> rL.getDriveMotor().getDutyCycle().getValueAsDouble());
//        logger.addMetric("RR_Drive_Command", () -> rR.getDriveMotor().getDutyCycle().getValueAsDouble());
//
//        logger.addMetric("FL_Turn_Command", () -> fL.getTurnMotor().getAppliedOutput());
//        logger.addMetric("FR_Turn_Command", () -> fR.getTurnMotor().getAppliedOutput());
//        logger.addMetric("RL_Turn_Command", () -> rL.getTurnMotor().getAppliedOutput());
//        logger.addMetric("RR_Turn_Command", () -> rR.getTurnMotor().getAppliedOutput());
//
//        logger.addMetric("FL_Turn_Velocity", () -> fL.getTurnMotor().getEncoder().getVelocity());
//        logger.addMetric("FR_Turn_Velocity", () -> fR.getTurnMotor().getEncoder().getVelocity());
//        logger.addMetric("RL_Turn_Velocity", () -> rL.getTurnMotor().getEncoder().getVelocity());
//        logger.addMetric("RR_Turn_Velocity", () -> rR.getTurnMotor().getEncoder().getVelocity());
    }

    /**
     * Periodic function
     * - constantly update the odometry
     */
    @Override
    public void periodic() {
        odometry.update(
            Robot.pigeon.getRotation2d(),
            getPosition()
        );

        if (Robot.fullDashboard) {
            updateShuffleboardDebug("Swerve");
        }

        //TODO SwerveAuto can remove after PID constants are finalized and autos are running well
//        System.out.println(  "X "+ String.format("%3.2f", Robot.swervetrain.getPose().getX()) +
//                           "  Y "+ String.format("%3.2f", Robot.swervetrain.getPose().getY()) +
//                           "  R "+ String.format("%3.2f", Robot.swervetrain.getPose().getRotation().getDegrees()));
    }

    /**
     * Reset the odometry to a given pose
     * @param pose the pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Robot.pigeon.getRotation2d(), new SwerveModulePosition[]{fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()}, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeed() {
        // Example module states
        var frontLeftState = new SwerveModuleState(fL.getDriveVelocity(), Rotation2d.fromDegrees(fL.encoderDegrees()));
        var frontRightState = new SwerveModuleState(fR.getDriveVelocity(), Rotation2d.fromDegrees(fR.encoderDegrees()));
        var rearLeft = new SwerveModuleState(rL.getDriveVelocity(), Rotation2d.fromDegrees(rL.encoderDegrees()));
        var rearRight = new SwerveModuleState(rR.getDriveVelocity(), Rotation2d.fromDegrees(rR.encoderDegrees()));

// Convert to chassis speeds
        return Constants.Drivetrain.DRIVE_KINEMATICS.toChassisSpeeds(
                frontLeftState, frontRightState, rearLeft, rearRight);
    }

    /**
     * Drive the bot with given params - always field relative
     * @param x dForward
     * @param y dLeft
     * @param rot dRot
     */
    public void drive(double x, double y, double rot) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(
                        drivePace.getIsFieldRelative() ?
                                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, Robot.pigeon.getRotation2d())
                                : new ChassisSpeeds(x, y, rot));

        SwerveModuleState[] actualStates = { fL.getState(), fR.getState(), rL.getState(), rR.getState()};
        targetPublisher.set(swerveModuleStates);
        actualPublisher.set(actualStates);
        setModuleStates(swerveModuleStates);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a SwerveModuleState array
     * @param desiredStates SwerveModuleState array of desired states for each of the modules
     * @implNote Only for use in the SwerveDrivetrain class and the RobotTrajectory Singleton, for any general use {@link SwerveDrivetrainMk4#drive(double, double, double)}
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, drivePace.getValue()
        );

        fL.setDesiredState(desiredStates[0]);
        fR.setDesiredState(desiredStates[1]);
        rL.setDesiredState(desiredStates[2]);
        rR.setDesiredState(desiredStates[3]);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a ChassisSpeeds class
     * @param chassisSpeeds Field Relative ChassisSpeeds to apply to wheel speeds
     * @implNote Use only in {@link SwerveDrivetrainMk4} or {@link team.gif.lib.RobotTrajectory}
     */
    public void setModuleChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, drivePace.getValue()
        );

        fL.setDesiredState(swerveModuleStates[0]);
        fR.setDesiredState(swerveModuleStates[1]);
        rL.setDesiredState(swerveModuleStates[2]);
        rR.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Reset the position of each of the wheels so that they all are pointing straight forward
     */
    public void resetEncoders() {
        fL.resetDriveEncoders();
        fR.resetDriveEncoders();
        rL.resetDriveEncoders();
        rR.resetDriveEncoders();
    }

    /**
     * Reset the pigeon heading
     */
    public void resetHeading() {
        Robot.pigeon.resetPigeonPosition();
    }


    /**
     * Get the pigeon heading
     * @return The pigeon heading in degrees
     */
    public Rotation2d getHeading() {
        return Robot.pigeon.getRotation2d();
    }

    /**
     * Get the current pose of the robot
     * @return The current pose of the robot (Pose2D)
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Stop all of the modules
     */
    public void stopModules() {
        fL.stop();
        fR.stop();
        rR.stop();
        rL.stop();
    }

    /**
     * Get the current position of each of the swerve modules
     * @return An array in form fL -> fR -> rL -> rR of each of the module positions
     */
    public SwerveModulePosition[] getPosition() {

        return new SwerveModulePosition[] {fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()};
    }

    /**
     * Reset the drive encoders
     */
    public void resetDriveEncoders() {
        fL.resetDriveEncoders();
        fR.resetDriveEncoders();
        rL.resetDriveEncoders();
        rR.resetDriveEncoders();
    }

    /**
     * Get the current heading of the robot
     * @return the heading of the robot in degrees
     */
    public double getRobotHeading() {
        return Robot.pigeon.getCompassHeading();
    }

    /**
     * set the drivePace settings for the drivebase
     * @param drivePace the drivePace to set
     */
    public void setDrivePace(drivePace drivePace) {
        this.drivePace = drivePace;
    }

    /**
     * Get the current drivePace settings
     * @return the current drivePace settings
     */
    public static drivePace getDrivePace() {
        return drivePace;
    }

    public double getPoseX() {
        return getPose().getX();
    }

    public double getPoseY() {
        return getPose().getY();
    }

    public void updateShuffleboardDebug(String shuffleboardTabName) {

        SmartDashboard.putNumber(shuffleboardTabName + "/FL Heading", fL.getTurningHeadingDegrees());
        SmartDashboard.putNumber(shuffleboardTabName + "/FR Heading", fR.getTurningHeadingDegrees());
        SmartDashboard.putNumber(shuffleboardTabName + "/RL Heading", rL.getTurningHeadingDegrees());
        SmartDashboard.putNumber(shuffleboardTabName + "/RR Heading", rR.getTurningHeadingDegrees());

        SmartDashboard.putNumber(shuffleboardTabName + "/FR Raw Degrees", fR.encoderDegrees());
        SmartDashboard.putNumber(shuffleboardTabName + "/FL Raw Degrees", fL.encoderDegrees());
        SmartDashboard.putNumber(shuffleboardTabName + "/RR Raw Degrees", rR.encoderDegrees());
        SmartDashboard.putNumber(shuffleboardTabName + "/RL Raw Degrees", rL.encoderDegrees());

        SmartDashboard.putNumber(shuffleboardTabName + "/FL Raw Encoder", fLEncoder.getTicks());
        SmartDashboard.putNumber(shuffleboardTabName + "/FR Raw Encoder", fREncoder.getTicks());
        SmartDashboard.putNumber(shuffleboardTabName + "/RL Raw Encoder", rLEncoder.getTicks());
        SmartDashboard.putNumber(shuffleboardTabName + "/RR Raw Encoder", rREncoder.getTicks());

        SmartDashboard.putNumber(shuffleboardTabName + "/FR Raw Radians", fR.getTurningHeading());
        SmartDashboard.putNumber(shuffleboardTabName + "/FL Raw Radians", fL.getTurningHeading());
        SmartDashboard.putNumber(shuffleboardTabName + "/RR Raw Radians", rR.getTurningHeading());
        SmartDashboard.putNumber(shuffleboardTabName + "/RL Raw Radians", rL.getTurningHeading());

        SmartDashboard.putNumber(shuffleboardTabName + "/FL Drive Encoder", fLDriveMotor.getPosition());
        SmartDashboard.putNumber(shuffleboardTabName + "/FR Drive Encoder", fRDriveMotor.getPosition());
        SmartDashboard.putNumber(shuffleboardTabName + "/RL Drive Encoder", rLDriveMotor.getPosition());
        SmartDashboard.putNumber(shuffleboardTabName + "/RR Drive Encoder", rRDriveMotor.getPosition());

        //TODO: Add target to shuffleboard
    }
}
