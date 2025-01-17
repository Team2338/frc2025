package team.gif.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class UiSmartDashboard {

    private NetworkTable networkTable;
    private NetworkTableEntry motorTempEntry;
    private NetworkTableEntry shootPercEntry;
    private NetworkTableEntry indexPercEntry;

    /**
     *  Widgets (e.g. gyro),
     *  buttons (e.g. SmartDashboard.putData("Reset", new ResetHeading()); ),
     *  and Chooser options (e.g. auto mode)
     *
     *  Placed in 2338-dashboard network table
     *  After SmartDashboard loads for the first time, move items from network table onto Dashboard tab
     *  and save file as "YYYY shuffleboard layout.json"
     */
    public UiSmartDashboard() {
        // create network table and entries for items to be displayed on main dashboard
        networkTable = NetworkTableInstance.getDefault().getTable("2338-dashboard");
        motorTempEntry = networkTable.getEntry("Motor Temp");
        shootPercEntry = networkTable.getEntry("Shoot %");
        indexPercEntry = networkTable.getEntry("Index %");

        // Set initial values for dashboard input fields
        shootPercEntry.setDefaultDouble(0.5);
    }

    /**
     * Values which are updated periodically should be placed here
     *
     * Convenient way to format a number is to use putString w/ format:
     *     elevatorPosEntry.setString(String.format("%11.2f", Elevator.getPosition());
     */
    public void updateUI() {
        // update the 2338 dashboard (i.e. main dashboard) network table entries
        motorTempEntry.setBoolean(Robot.diagnostics.getAnyMotorTempHot());
    }

    public double getUiShootPerc() {
        return shootPercEntry.getDouble(0);
    }

    public double getUiIndexPerc() {
        return indexPercEntry.getDouble(0);
    }
}
