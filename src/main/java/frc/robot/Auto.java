package frc.robot;
//End of theirs
import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Swerve;

public class Auto {
    private final SendableChooser<String> pathChooser = new SendableChooser<>();

    public Auto()
    {
        pathChooser.addOption("--- Auto Routine ---", null);
        for (String pathName : getPathnames()) {
            pathChooser.addOption(pathName, pathName);
        }
        SmartDashboard.putData("Autonomous routine", pathChooser);
    }

    public Command getAutoCommand(Swerve s_Swerve)
    {
        if (pathChooser.getSelected() != null)
            return buildCommand(s_Swerve, pathChooser.getSelected());
        else return null;
    }

    public Command buildCommand(Swerve s_Swerve, String pathName) {
        if (pathName == null)
            return null;

        // List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(config.path);
        return Commands.runOnce(() -> {
            DriverStation.reportWarning("Running auto command built from path: " + pathName, false);
        })
        .andThen(new PathPlannerAuto(pathName));
    }

    private static List<String> getPathnames() {
        return Stream.of(new File(Filesystem.getDeployDirectory(), "pathplanner/autos").listFiles())
                .filter(file -> !file.isDirectory())
                .filter(file -> file.getName().matches(".*\\.auto"))
                .map(File::getName)
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .sorted()
                .collect(Collectors.toList());
    }
}