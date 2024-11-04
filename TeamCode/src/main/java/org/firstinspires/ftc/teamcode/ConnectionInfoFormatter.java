package org.firstinspires.ftc.teamcode;


import android.util.Log;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Utility class for formatting connection information.
 */
public class ConnectionInfoFormatter {
    /**
     * Reformat the raw connection information string by extracting the module number, bus number, and port number.
     * Transforms "module X; bus Y; port Z" to "HubName I2C Bus Y Port Z".
     * For example, "module 2; bus 1; port 3" becomes "Exp. Hub I2C Bus 1 Port 3".
     *
     * @param rawConnectionInfo The original connection info string (e.g., "module 2; bus 1; port 3").
     * @return The reformatted connection info string (e.g., "Exp. Hub I2C Bus 1 Port 3").
     */
    public static String reformatConnectionInfo(String rawConnectionInfo) {
        if (rawConnectionInfo == null || rawConnectionInfo.isEmpty()) {
            return "Unknown Connection";
        }

        // Enhanced regular expression to extract module number, bus number, and port number
        // Allows for:
        // - Case insensitivity
        // - Optional spaces
        // - Different delimiters (e.g., ";", ",")
        Pattern pattern = Pattern.compile(
                "module\\s*(\\d+)\\s*[;,]\\s*bus\\s*(\\d+)(?:\\s*[;,]\\s*port\\s*(\\d+))?",
                Pattern.CASE_INSENSITIVE
        );
        Matcher matcher = pattern.matcher(rawConnectionInfo.trim());

        if (matcher.find()) { // Use find() instead of matches() for partial matches
            try {
                int moduleNumber = Integer.parseInt(matcher.group(1));
                int busNumber = Integer.parseInt(matcher.group(2));
                int portNumber = -1; // Default value if port number is not found

                if (matcher.group(3) != null) {
                    portNumber = Integer.parseInt(matcher.group(3));
                }

                // Map module numbers to hub names
                String hubName = mapModuleToHub(moduleNumber);
                // Since all buses are I2C, we can directly assign the bus type
                String busType = "I2C";

                // Build the formatted connection info string
                String formattedConnection = String.format("%s %s Bus %d", hubName, busType, busNumber);

                if (portNumber != -1) {
                    formattedConnection += String.format(" Port %d", portNumber);
                }

                return formattedConnection;
            } catch (NumberFormatException e) {
                // Logging the error for debugging purposes
                Log.e("ConnectionInfoFormatter", "Failed to parse numbers from connection info: " + rawConnectionInfo, e);
                return "Unknown Connection Format";
            }
        } else {
            // Alternative Parsing: Split by delimiters and extract numbers
            String[] parts = rawConnectionInfo.split("[,;]");
            if (parts.length >= 2) { // Allow for missing port number
                String modulePart = parts[0].trim().toLowerCase();
                String busPart = parts[1].trim().toLowerCase();
                String portPart = parts.length >= 3 ? parts[2].trim().toLowerCase() : "";

                try {
                    int moduleNumber = Integer.parseInt(modulePart.replaceAll("[^0-9]", ""));
                    int busNumber = Integer.parseInt(busPart.replaceAll("[^0-9]", ""));
                    int portNumber = portPart.startsWith("port") ? Integer.parseInt(portPart.replaceAll("[^0-9]", "")) : -1;

                    String hubName = mapModuleToHub(moduleNumber);
                    String busType = "I2C";

                    String formattedConnection = String.format("%s %s Bus %d", hubName, busType, busNumber);

                    if (portNumber != -1) {
                        formattedConnection += String.format(" Port %d", portNumber);
                    }

                    return formattedConnection;
                } catch (NumberFormatException e) {
                    // Logging the error for debugging purposes
                    Log.e("ConnectionInfoFormatter", "Failed to parse connection info: " + rawConnectionInfo, e);
                    return "Unknown Connection Format";
                }
            }

            // If unable to parse, log the unexpected format
            Log.w("ConnectionInfoFormatter", "Unexpected connection info format: " + rawConnectionInfo);
            return "Unknown Connection Format";
        }
    }

    /**
     * Helper method to extract the port number from the connection info string.
     * Assumes the connection info contains "port X" where X is the port number.
     *
     * @param connectionInfo The connection info string from getConnectionInfo().
     * @return The port number as an integer, or -1 if not found.
     */
    public static int extractPortNumber(String connectionInfo) {
        if (connectionInfo == null || connectionInfo.isEmpty()) {
            return -1;
        }

        // Split the connection info by semicolons or commas
        String[] parts = connectionInfo.split("[,;]");
        for (String part : parts) {
            part = part.trim().toLowerCase();
            if (part.startsWith("port")) {
                // Handle "port X" or "portX"
                String portStr = part.replaceAll("[^0-9]", ""); // Remove non-numeric characters
                try {
                    return Integer.parseInt(portStr);
                } catch (NumberFormatException e) {
                    // Parsing error
                    Log.e("ConnectionInfoFormatter", "Failed to parse port number from connection info: " + connectionInfo, e);
                    return -1;
                }
            }
        }
        // Port number not found
        return -1;
    }

    /**
     * Maps module numbers to their corresponding hub names.
     *
     * @param moduleNumber The module number extracted from the connection info.
     * @return The hub name corresponding to the module number.
     */
    private static String mapModuleToHub(int moduleNumber) {
        switch (moduleNumber) {
            case 1:
                return "Control Hub"; // Assuming module 1 is Control Hub
            case 2:
                return "Exp. Hub"; // Assuming module 2 is Expansion Hub
            // Add more cases if there are additional modules
            default:
                return "Unknown Hub";
        }
    }
}
