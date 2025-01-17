// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.lib.LimelightHelpers;
import team.gif.robot.commands.drivetrainPbot.DriveSwerve;
import team.gif.robot.subsystems.Diagnostics;
import team.gif.robot.subsystems.Shooter;
import team.gif.robot.subsystems.SwerveDrivetrainMk3;
import team.gif.robot.subsystems.SwerveDrivetrainMk4;
import team.gif.robot.subsystems.drivers.Limelight;
import team.gif.robot.subsystems.drivers.Pigeon2_0;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

  private static RobotContainer robotContainer;
  public static Diagnostics diagnostics;
  public static OI oi;
  public static UI ui;
  public static UiSmartDashboard uiSmartDashboard;

    //Devices
    public static Pigeon2_0 pigeon;
    public static SwerveDrivetrainMk3 swerveDrive;
    //  public static SwerveDrivetrainMk4 swerveDrive;
    public static Limelight limelightCollector;
    public static Limelight limelightShooter;
    public static Shooter shooter;

    public static final boolean fullDashboard = true;

    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    public Robot() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        pigeon = new Pigeon2_0(RobotMap.PIGEON_ID);
        limelightCollector = new Limelight("limelight-collect");
        limelightShooter = new Limelight("limelight-shooter");
        swerveDrive = new SwerveDrivetrainMk3();
        swerveDrive.setDefaultCommand(new DriveSwerve());
        //  swerveDrive = new SwerveDrivetrainMk4();
        shooter= new Shooter();
        diagnostics = new Diagnostics();

        oi = new OI();
        ui = new UI();
        uiSmartDashboard = new UiSmartDashboard();
        pigeon.addToShuffleboard("FRC 2025", "Heading");
        autonomousCommand = new PathPlannerAuto("Straight Line");
        //    try {
        //      autonomousCommand = AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path"));
        //    } catch (Exception e) {
        //      DriverStation.reportError("PathPlannerError " + e.getMessage(), e.getStackTrace());
        //      autonomousCommand =  Commands.none();
        //    }
    }

    /**
    * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
    * that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
    * SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        uiSmartDashboard.updateUI();

        //Vision Localization
    //        limelightCollector.setRobotOrientation(pigeon.getCompassHeading(), 0, 0, 0, 0, 0);
        limelightCollector.setRobotOrientation(pigeon.getHeading(), 0, 0, 0, 0, 0);
        limelightShooter.setRobotOrientation(pigeon.getHeading(), 0, 0, 0, 0, 0);
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        shooter.moveIndexerFromShuffleboard();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
