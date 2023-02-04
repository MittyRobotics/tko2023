package com.github.mittyrobotics;

import edu.wpi.first.networktables.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class LoggerInterface {
    private final NetworkTableInstance nt;
    private final NetworkTable table;
    private final HashMap<String, StringPublisher> pubs = new HashMap<>();

    public LoggerInterface() {
        nt = NetworkTableInstance.getDefault();
        nt.startServer();
//        nt.setServerTeam(1351);
//        nt.startDSClient();

        table = nt.getTable("Log");

        System.out.println(nt.isConnected());
    }

    public void put(String key, Object value) {
        if (pubs.containsKey(key)) {
            pubs.get(key).set(String.valueOf(value), System.currentTimeMillis());
        } else {
            StringTopic dt = table.getStringTopic(key);
            StringPublisher pub = dt.publish(PubSubOption.keepDuplicates(true));
            pub.set(String.valueOf(value), System.currentTimeMillis());
            pubs.put(key, pub);
        }
    }
}
