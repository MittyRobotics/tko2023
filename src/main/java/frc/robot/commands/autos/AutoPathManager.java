package frc.robot.commands.autos;

import frc.robot.util.autonomous.SwervePath;

import java.util.HashMap;

import static frc.robot.commands.autos.AutoPathManager.PathName.*;

public class AutoPathManager {
    HashMap<PathName, SwervePath> paths;

    public AutoPathManager(int startTag) {
        paths = new HashMap<>();

        paths.put(TAXI, new SwervePath(

        ));
        paths.put(TO_FIRST_PIECE, new SwervePath(

        ));
        paths.put(TO_SECOND_PIECE, new SwervePath(

        ));
        paths.put(TO_THIRD_PIECE, new SwervePath(

        ));
        paths.put(BALANCE_FAR_SIDE, new SwervePath(

        ));
        paths.put(BALANCE_COMMUNITY_SIDE, new SwervePath(

        ));
    }

    enum PathName {
        TAXI,
        TO_FIRST_PIECE,
        TO_SECOND_PIECE,
        TO_THIRD_PIECE,
        BALANCE_FAR_SIDE,
        BALANCE_COMMUNITY_SIDE
    }
}
