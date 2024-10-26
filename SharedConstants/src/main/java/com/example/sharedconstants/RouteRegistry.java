// RouteRegistry.java
package com.example.sharedconstants;

import com.example.sharedconstants.Routes.Routes;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

/**
 * Central registry for all autonomous routes.
 * Routes self-register by calling registerRoute with their corresponding enum identifier.
 */
public class RouteRegistry {
    // Map to hold the association between RoutesToRun enums and their constructors
    private static final Map<RoutesToRun, Function<RobotAdapter, Routes>> registry = new HashMap<>();

    /**
     * Registers a route with the specified RoutesToRun enum.
     *
     * @param routeToRun  The enum constant representing the route.
     * @param constructor A function that takes a RobotAdapter and returns a Routes instance.
     */
    public static void registerRoute(RoutesToRun routeToRun, Function<RobotAdapter, Routes> constructor) {
        registry.put(routeToRun, constructor);
    }

    /**
     * Retrieves a Routes instance based on the RoutesToRun enum.
     *
     * @param routeToRun The enum constant representing the desired route.
     * @param adapter    The RobotAdapter instance required for route construction.
     * @return A Routes instance corresponding to the routeToRun.
     * @throws IllegalArgumentException if the routeToRun is not registered.
     */
    public static Routes getRoute(RoutesToRun routeToRun, RobotAdapter adapter) {
        Function<RobotAdapter, Routes> constructor = registry.get(routeToRun);
        if (constructor != null) {
            return constructor.apply(adapter);
        }
        throw new IllegalArgumentException("No route found with name: " + routeToRun);
    }
}
