package com.example.meepmeeptesting.ColorSchemes;

import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import java.awt.Color;

public class CustomColorSchemeLightRed extends ColorScheme {

    @Override
    public boolean isDark() {
        return false;
    }

    @Override
    public Color getBOT_BODY_COLOR() {
        return new Color(255, 69, 69); // True Light Red
    }

    @Override
    public Color getBOT_WHEEL_COLOR() {
        return new Color(200, 30, 30); // Darker Light Red
    }

    @Override
    public Color getBOT_DIRECTION_COLOR() {
        return new Color(220, 20, 20); // Medium True Red
    }

    @Override
    public Color getAXIS_X_COLOR() {
        return new Color(0, 0, 0);
    }

    @Override
    public Color getAXIS_Y_COLOR() {
        return new Color(0, 0, 0);
    }

    @Override
    public double getAXIS_NORMAL_OPACITY() {
        return 0.2;
    }

    @Override
    public double getAXIS_HOVER_OPACITY() {
        return 0.4;
    }

    @Override
    public Color getTRAJECTORY_PATH_COLOR() {
        return new Color(255, 100, 100); // Light Red
    }

    @Override
    public Color getTRAJECTORY_TURN_COLOR() {
        return new Color(240, 80, 80); // Soft Light Red
    }

    @Override
    public Color getTRAJECTORY_MARKER_COLOR() {
        return new Color(255, 102, 102); // Light Red
    }

    @Override
    public Color getTRAJECTORY_SLIDER_BG() {
        return new Color(192, 192, 192); // Light Gray
    }

    @Override
    public Color getTRAJECTORY_SLIDER_FG() {
        return new Color(220, 50, 50); // Medium Red
    }

    @Override
    public Color getTRAJECTORY_TEXT_COLOR() {
        return new Color(0, 0, 0); // Black
    }

    @Override
    public Color getUI_MAIN_BG() {
        return new Color(245, 245, 245); // White Smoke
    }
}
