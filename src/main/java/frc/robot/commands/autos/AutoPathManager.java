package frc.robot.commands.autos;

import frc.robot.util.autonomous.SwervePath;

import java.util.HashMap;

import static frc.robot.commands.autos.AutoPathManager.PathName.*;

public class AutoPathManager {
    HashMap<PathName, SwervePath> paths;

    public AutoPathManager() {
        paths = new HashMap<>();

        paths.put(LOW_TAXI, new SwervePath(

        ));

        paths.put(LOW_TO_FIRST_PIECE, new SwervePath(

        ));

        paths.put(LOW_TO_SECOND_PIECE, new SwervePath(

        ));

        paths.put(LOW_TO_THIRD_PIECE, new SwervePath(

        ));

        paths.put(LOW_BALANCE_FAR_SIDE, new SwervePath(

        ));



        paths.put(HIGH_TAXI, new SwervePath(

        ));

        paths.put(HIGH_TO_FIRST_PIECE, new SwervePath(

        ));

        paths.put(HIGH_TO_SECOND_PIECE, new SwervePath(

        ));

        paths.put(HIGH_TO_THIRD_PIECE, new SwervePath(

        ));

        paths.put(HIGH_BALANCE_FAR_SIDE, new SwervePath(

        ));



        paths.put(BALANCE_COMMUNITY_SIDE, new SwervePath(

        ));
    }

    enum PathName {
        LOW_TAXI,
        LOW_TO_FIRST_PIECE,
        LOW_TO_SECOND_PIECE,
        LOW_TO_THIRD_PIECE,
        LOW_BALANCE_FAR_SIDE,

        HIGH_TAXI,
        HIGH_TO_FIRST_PIECE,
        HIGH_TO_SECOND_PIECE,
        HIGH_TO_THIRD_PIECE,
        HIGH_BALANCE_FAR_SIDE,

        BALANCE_COMMUNITY_SIDE
    }
}
