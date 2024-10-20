package com.example.meepmeeptesting.ColorSchemes;

import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import java.awt.Color;

public class CustomColorSchemeDarkBlue extends ColorScheme {

    @Override
    public boolean isDark() {
        return true;
    }

    @Override
    public Color getBOT_BODY_COLOR() {
        return new Color(0, 0, 139); // Dark Blue
    }

    @Override
    public Color getBOT_WHEEL_COLOR() {
        return new Color(0, 0, 205); // Medium Blue
    }

    @Override
    public Color getBOT_DIRECTION_COLOR() {
        return new Color(25, 25, 112); // Midnight Blue
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
        return new Color(0, 0, 255); // Blue
    }

    @Override
    public Color getTRAJECTORY_TURN_COLOR() {
        return new Color(70, 130, 180); // Steel Blue
    }

    @Override
    public Color getTRAJECTORY_MARKER_COLOR() {
        return new Color(30, 144, 255); // Dodger Blue
    }

    @Override
    public Color getTRAJECTORY_SLIDER_BG() {
        return new Color(105, 105, 105); // Dim Gray
    }

    @Override
    public Color getTRAJECTORY_SLIDER_FG() {
        return new Color(0, 0, 139); // Dark Blue
    }

    @Override
    public Color getTRAJECTORY_TEXT_COLOR() {
        return new Color(255, 255, 255); // White
    }

    @Override
    public Color getUI_MAIN_BG() {
        return new Color(40, 40, 60); // Very Dark Blue
    }
}
