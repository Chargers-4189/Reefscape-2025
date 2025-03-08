package frc.util;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GetAprilTagRotation {

    private static HashMap<Integer, Integer> reefTags = new HashMap<>();
    private static HashMap<Integer, Integer> stationTags = new HashMap<>();


    public static Rotation2d getReefTagAngle(int tagId) {
        return new Rotation2d(Units.degreesToRadians((reefTags.get(tagId))));
    }
    public static Rotation2d getStationTagAngle(int tagId) {
        return new Rotation2d(Units.degreesToRadians((stationTags.get(tagId))));
    }

    public static void initialize() {
        reefTags.put(17, 60);
        reefTags.put(18, 0);
        reefTags.put(19, -60);
        reefTags.put(20, -120);
        reefTags.put(21, 180);
        reefTags.put(22, 120);

        reefTags.put(8, 60);
        reefTags.put(7, 0);
        reefTags.put(6, -60);
        reefTags.put(11, -120);
        reefTags.put(10, 180);
        reefTags.put(9, 120);

        stationTags.put(1, 126);
        stationTags.put(2, -126);
        stationTags.put(12, -126);
        stationTags.put(13, 126);
    }
}
