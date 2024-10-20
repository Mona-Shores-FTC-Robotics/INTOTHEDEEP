package com.example.meepmeeptesting.ColorSchemes;

import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import java.awt.Color;

public class CustomColorSchemeLightBlue extends ColorScheme {

    @Override
    public boolean isDark() {
        return false;
    }

    @Override
    public Color getBOT_BODY_COLOR() {
        return new Color(100, 149, 237); // Cornflower Blue
    }

    @Override
    public Color getBOT_WHEEL_COLOR() {
        return new Color(70, 130, 180); // Steel Blue
    }

    @Override
    public Color getBOT_DIRECTION_COLOR() {
        return new Color(100, 149, 237); // Cornflower Blue
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
        return new Color(100, 149, 237); // Cornflower Blue
    }

    @Override
    public Color getTRAJECTORY_TURN_COLOR() {
        return new Color(173, 216, 230); // Light Blue
    }

    @Override
    public Color getTRAJECTORY_MARKER_COLOR() {
        return new Color(176, 224, 230); // Powder Blue
    }

    @Override
    public Color getTRAJECTORY_SLIDER_BG() {
        return new Color(169, 169, 169); // Dark Gray
    }

    @Override
    public Color getTRAJECTORY_SLIDER_FG() {
        return new Color(135, 206, 250); // Sky Blue
    }

    @Override
    public Color getTRAJECTORY_TEXT_COLOR() {
        return new Color(0, 0, 0); // Black
    }

    @Override
    public Color getUI_MAIN_BG() {
        return new Color(220, 220, 250); // Lighter Blue Gray
    }
}
