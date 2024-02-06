package frc.robot;
//theirs
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import org.json.simple.parser.JSONParser;
import org.json.simple.JSONObject;


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
import frc.robot.oldcommands.*;
import frc.robot.oldsubsystems.*;

public class Auto {
    private final HashMap<String, Command> eventMap = new HashMap<>();

    private final SendableChooser<String> pathChooser = new SendableChooser<>();
    private final SendableChooser<Boolean> pieceChooser = new SendableChooser<>();
    private final SendableChooser<Command> quickChooser = new SendableChooser<>();

    private final double firstArmTimeout = 2.5;
    private AutoConfig activeConfig;

    public static class AutoConfig {
        private final String path;

        public AutoConfig(String path) {
            this.path = path;
        }
    }

    private final SortedMap<String, AutoConfig> quickpicks = new TreeMap<>(Map.ofEntries(
    //    Map.entry("[Clear-Y] 2 HH + balance [outer]", new AutoConfig("SS-PlaceCone,pick,place,balance")),
      //  Map.entry("[Clear-Y] 2 HH + balance [outer]", new AutoConfig("SS-PlaceCone,pick,place,balance")),
    ));

    public Auto(Swerve s_Swerve, RobotPosition s_RobotPosition) {
        // eventMap.put("Move arm to second", new ArmCommand(s_Arm, () -> activeConfig.place2));
        // pieceChooser.addOption("--- Initial Piece ---", null);
        // pieceChooser.setDefaultOption("Cube", Boolean.TRUE);
        // SmartDashboard.putData("Starting game piece", pieceChooser);

        pathChooser.addOption("--- Auto Routine ---", null);
        pathChooser.setDefaultOption("No auto (face intake away)", null);
        for (String pathName : getPathnames()) {
            pathChooser.addOption(pathName, pathName);
        }
        SmartDashboard.putData("Autonomous routine", pathChooser);

        quickChooser.addOption("--- Auto Quick Pick ---", null);
        quickChooser.setDefaultOption("Manual selection", null);
        for (String name : quickpicks.keySet()) {
          quickChooser.addOption(name, buildCommand(quickpicks.get(name)));
        }
        SmartDashboard.putData("Quick Picks", quickChooser);
    }

    public Command getAutoCommand() {
        Command autoCommand = quickChooser.getSelected();

        if (autoCommand == null) {
            AutoConfig config = new AutoConfig(pathChooser.getSelected());
            autoCommand = buildCommand(config);
        }
        return autoCommand;
    }

    public Command buildCommand(AutoConfig config) {
        if (config == null || config.path == null)
            return null;

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(config.path, PathPlanner.getConstraintsFromPath(config.path));
        return Commands.runOnce(() -> {
            DriverStation.reportWarning("Running auto command built from path: " + config.path, false);
            activeConfig = config;
        }).andThen(AutoBuilder.fullAuto(pathGroup));
    }

    private static List<String> getPathnames() {
        return Stream.of(new File(Filesystem.getDeployDirectory(), "pathplanner").listFiles())
                .filter(file -> !file.isDirectory())
                .filter(file -> file.getName().matches(".*\\.path"))
                .map(File::getName)
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .sorted()
                .collect(Collectors.toList());
    }
}