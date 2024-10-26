// RouteLoader.java
package com.example.sharedconstants;

/**
 * Responsible for loading all route classes to trigger their self-registration.
 */
public class RouteLoader {
    public static void loadAllRoutes() {
        try {
            // NET Routes
            Class.forName("com.example.sharedconstants.Routes.NET.NET_Score_1_Specimen_Preload");
            Class.forName("com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload");
        } catch (ClassNotFoundException e) {
            throw new RuntimeException("Failed to load route classes", e);
        }
    }
}
