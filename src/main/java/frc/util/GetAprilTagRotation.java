package frc.util;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GetAprilTagRotation {

    private static HashMap<Integer, Integer> map = new HashMap<>();


    public static Rotation2d getAprilTagRotation(int tagId) {
        return new Rotation2d(Units.degreesToRadians((map.get(tagId))));
    }

    public static void initialize() {
        map.put(17, 60);
        map.put(18, 0);
        map.put(19, -60);
        map.put(20, -120);
        map.put(21, 180);
        map.put(22, 120);

        map.put(8, 60);
        map.put(7, 0);
        map.put(6, -60);
        map.put(11, -120);
        map.put(10, 180);
        map.put(9, 120);
    }
}
