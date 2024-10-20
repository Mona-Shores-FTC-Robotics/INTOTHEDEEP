package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.example.sharedconstants.Routes.Routes;
import org.reflections.Reflections;
import org.reflections.scanners.SubTypesScanner;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class AutoRouteSelector {
    private static final String ROUTES_PACKAGE = "com.example.sharedconstants.Routes";

    public AutoRouteSelector() {
    }

    // Get a list of all available routes
    public List<Class<? extends Routes>> getAvailableRoutes() {
        Reflections reflections = new Reflections(ROUTES_PACKAGE, new SubTypesScanner(false));
        Set<Class<? extends Routes>> classes = reflections.getSubTypesOf(Routes.class);
        return new ArrayList<>(classes);
    }
}
