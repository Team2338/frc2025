package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class GrabCoral extends Command {

    public GrabCoral() {
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        boolean isFirstSensorActive = Robot.shooter.getSecondSensorState();
        boolean isSecondSensorActive = Robot.shooter.getFirstSensorState();

        if (!isSecondSensorActive && !isFirstSensorActive) {
            Robot.shooter.moveMotor(0);
        }

        if (isSecondSensorActive && !isFirstSensorActive) {
            Robot.shooter.moveMotor(Constants.SHOOTER_SPEED_GRAB_PERCENT);
        }

        if (isFirstSensorActive) {
            Robot.shooter.moveMotor(0);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { Robot.shooter.moveMotor(0); }
}
