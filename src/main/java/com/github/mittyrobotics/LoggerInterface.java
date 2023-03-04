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

//        table.setDefaultValue("seen", NetworkTableValue.makeBoolean(true));

//        table = nt.getTable("SmartDasboard");


//        log = new DataLog();
//        nt = NetworkTableInstance.getDefault();
//        nt.startDSClient();
//        nt.setServerTeam(1351);
//        nt.setServer("172.22.11.2");
//        nt.startEntryDataLog(log, "", "");
//        table = nt.getDefault().getTable("Table");



//        log = new DataLog("log");
//        nt = NetworkTableInstance.create();
//        nt.setServer("");
//        nt.startConnectionDataLog(log, "a");
//        nt.startClient4("Client");
//        nt.setServerTeam(1351);
//        nt.startDSClient();
    }

//    public boolean connected() {
//        return nt.isConnected();
//    }



//    public DoubleTopic getTopic() {
//        return table.getDoubleTopic("Table");
//    }




//    public void put(String key, Object value) {
//        if (pubs.containsKey(key)) {
//            pubs.get(key).set(String.valueOf(value), System.currentTimeMillis());
//        } else {
//            StringTopic dt = table.getStringTopic(key);
//            StringPublisher pub = dt.publish(PubSubOption.keepDuplicates(true));
//            pub.set(String.valueOf(value), System.currentTimeMillis());
//            pubs.put(key, pub);
//        }
//    }

//    public static synchronized void putData(String key, Sendable data) {
//        Sendable sddata = tablesToData.get(key);
//        if (sddata == null || sddata != data) {
//            tablesToData.put(key, data);
//            NetworkTable dataTable = table.getSubTable(key);
//            SendableBuilderImpl builder = new SendableBuilderImpl();
//            builder.setTable(dataTable);
//            SendableRegistry.publish(data, builder);
//            builder.startListeners();
//            dataTable.getEntry(".name").setString(key);
//        }
//    }

//    public static NetworkTableEntry getEntry(String key) {
//        return table.getEntry(key);
//    }

//    public static double getNumber(String key, double defaultValue) {
//        return getEntry(key).getDouble(defaultValue);
//    }

    public JSONObject getGamePiece() throws JSONException {
        return new JSONObject(radarSub.get());
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
