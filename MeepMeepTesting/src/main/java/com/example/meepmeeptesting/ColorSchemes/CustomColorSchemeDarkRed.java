package com.example.meepmeeptesting.ColorSchemes;

import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import java.awt.Color;

public class CustomColorSchemeDarkRed extends ColorScheme {

    @Override
    public boolean isDark() {
        return true;
    }

    @Override
    public Color getBOT_BODY_COLOR() {
        return new Color(120, 0, 0); // Dark Crimson
    }

    @Override
    public Color getBOT_WHEEL_COLOR() {
        return new Color(80, 0, 0); // Very Dark Red
    }

    @Override
    public Color getBOT_DIRECTION_COLOR() {
        return new Color(139, 0, 0); // Dark Red color
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
        return new Color(102, 0, 0); // Deep Dark Red
    }

    @Override
    public Color getTRAJECTORY_TURN_COLOR() {
        return new Color(90, 0, 0); // Even Darker Red
    }

    @Override
    public Color getTRAJECTORY_MARKER_COLOR() {
        return new Color(139, 0, 0); // Dark Red
    }

    @Override
    public Color getTRAJECTORY_SLIDER_BG() {
        return new Color(169, 169, 169); // Dark Gray
    }

    @Override
    public Color getTRAJECTORY_SLIDER_FG() {
        return new Color(100, 0, 0); // Darker Red color
    }

    @Override
    public Color getTRAJECTORY_TEXT_COLOR() {
        return new Color(255, 255, 255); // White
    }

    @Override
    public Color getUI_MAIN_BG() {
        return new Color(105, 105, 105); // Dim Gray
    }
}
