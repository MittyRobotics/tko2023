package com.github.mittyrobotics;

import edu.wpi.first.networktables.*;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.*;

public class LoggerInterface {
    private static LoggerInterface instance;

    private final NetworkTableInstance nt;
    //    private static final Map<String, Sendable> tablesToData = new HashMap<>();
    private final NetworkTable table;
    private final DoubleArraySubscriber poseSub;
    private final StringSubscriber radarSub;

//    private final HashMap<String, StringPublisher> pubs = new HashMap<>();

    public static LoggerInterface getInstance() {
        if (instance == null) instance = new LoggerInterface();
        return instance;
    }


//    private final DataLog log;


    private LoggerInterface() {
        nt = NetworkTableInstance.getDefault();
        nt.setServerTeam(1351);
        nt.startDSClient();

        table = NetworkTableInstance.getDefault().getTable("dashboard");

        poseSub = table.getDoubleArrayTopic("pose").subscribe(new double[]{}, PubSubOption.keepDuplicates(true));
        radarSub = table.getStringTopic("gamepieces").subscribe(new String(), PubSubOption.keepDuplicates(true));

    }

    public void put(String key, Object o) {
        table.putValue(key, NetworkTableValue.makeString(String.valueOf(o)));
    }

    public JSONObject getGamePiece() {
        try {
            return new JSONObject(radarSub.get());
        } catch (Exception e) {
            return new JSONObject();
        }
    }

    public DoubleArraySubscriber getPoseSub() {
        return poseSub;
    }

    public void putDesiredCamera(int i) {
        table.putValue("idealcam", NetworkTableValue.makeInteger(i));
    }
    public void print() {

        System.out.println(table.getValue("gamepieces").getValue());

        for (double[] val : poseSub.readQueueValues()) {
            System.out.println("pose changed value " + Arrays.toString(val));
        }

//        if(!table.getValue("seen").getBoolean()) {
//            double[] pose_x = table.getValue("pose_x").getDoubleArray();
//            double[] pose_y = table.getValue("pose_y").getDoubleArray();
//            double[] pose_z = table.getValue("pose_z").getDoubleArray();
//            double[] pose_time = table.getValue("pose_time").getDoubleArray();
//
//            System.out.println("Pose X: " + Arrays.toString(pose_x));
//            System.out.println("Pose Y: " + Arrays.toString(pose_y));
//            System.out.println("Pose Z: " + Arrays.toString(pose_z));
//            System.out.println("Pose Time: " + Arrays.toString(pose_time));
//
//            table.putValue("seen", NetworkTableValue.makeBoolean(true));
//        }
//        for(String key : table.getKeys()) System.out.println(key + ": " + Arrays.toString(table.getValue(key).getDoubleArray()));
    }
}
