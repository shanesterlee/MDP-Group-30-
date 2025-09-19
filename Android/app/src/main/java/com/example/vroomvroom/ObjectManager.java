package com.example.vroomvroom;

import android.content.Context;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.TextView;

public class ObjectManager {

    public interface ObjectUpdateListener {
        void onObjectPositionChanged(String objectType, int x, int y, String direction);
        void onObjectDirectionChanged(String objectType, String direction);
        void onTargetUpdated(String objectType, int targetId);
        void onStatusMessage(String message);
    }

    private Context context;
    private ObjectUpdateListener listener;

    // Object position tracking
    private int object1X = -1, object1Y = -1;
    private int object2X = -1, object2Y = -1;
    private int object3X = -1, object3Y = -1;
    private int object4X = -1, object4Y = -1;
    private boolean object1Placed = false;
    private boolean object2Placed = false;
    private boolean object3Placed = false;
    private boolean object4Placed = false;

    // Object direction tracking
    private String object1Direction = "N";
    private String object2Direction = "N";
    private String object3Direction = "N";
    private String object4Direction = "N";

    // Object target ID tracking
    private int object1TargetId = 1;
    private int object2TargetId = 2;
    private int object3TargetId = 3;
    private int object4TargetId = 4;

    // View references
    private View object1DirectionView;
    private View object2DirectionView;
    private View object3DirectionView;
    private View object4DirectionView;
    private TextView object1TextView;
    private TextView object2TextView;
    private TextView object3TextView;
    private TextView object4TextView;

    public ObjectManager(Context context) {
        this.context = context;
    }

    public void setListener(ObjectUpdateListener listener) {
        this.listener = listener;
    }

    public void initializeViews(View rootView) {
        // Direction indicators
        object1DirectionView = rootView.findViewById(R.id.object1_direction);
        object2DirectionView = rootView.findViewById(R.id.object2_direction);
        object3DirectionView = rootView.findViewById(R.id.object3_direction);
        object4DirectionView = rootView.findViewById(R.id.object4_direction);

        // Text views
        object1TextView = rootView.findViewById(R.id.object1_number);
        object2TextView = rootView.findViewById(R.id.object2_number);
        object3TextView = rootView.findViewById(R.id.object3_number);
        object4TextView = rootView.findViewById(R.id.object4_number);

        // Initialize all objects to default state
        initializeDirectionIndicators();
        updateAllTargetDisplays();
    }

    // Position Management
    public void setObjectPosition(String objectType, int x, int y) {
        switch (objectType) {
            case "OBJECT1":
                object1X = x;
                object1Y = y;
                object1Placed = (x != -1 && y != -1);
                break;
            case "OBJECT2":
                object2X = x;
                object2Y = y;
                object2Placed = (x != -1 && y != -1);
                break;
            case "OBJECT3":
                object3X = x;
                object3Y = y;
                object3Placed = (x != -1 && y != -1);
                break;
            case "OBJECT4":
                object4X = x;
                object4Y = y;
                object4Placed = (x != -1 && y != -1);
                break;
        }

        if (listener != null) {
            listener.onObjectPositionChanged(objectType, x, y, getObjectDirection(objectType));
        }
    }

    public void resetObjectPosition(String objectType) {
        setObjectPosition(objectType, -1, -1);
    }

    // Direction Management
    public void setObjectDirection(String objectType, String direction) {
        switch (objectType) {
            case "OBJECT1":
                object1Direction = direction;
                break;
            case "OBJECT2":
                object2Direction = direction;
                break;
            case "OBJECT3":
                object3Direction = direction;
                break;
            case "OBJECT4":
                object4Direction = direction;
                break;
        }

        updateDirectionIndicator(objectType, direction);

        if (listener != null) {
            listener.onObjectDirectionChanged(objectType, direction);
        }
    }

    public String getObjectDirection(String objectType) {
        switch (objectType) {
            case "OBJECT1":
                return object1Direction;
            case "OBJECT2":
                return object2Direction;
            case "OBJECT3":
                return object3Direction;
            case "OBJECT4":
                return object4Direction;
            default:
                return "N";
        }
    }

    // Target ID Management
    public void setObjectTargetId(String objectType, int targetId) {
        if (targetId < 1 || targetId > 15) {
            if (listener != null) {
                listener.onStatusMessage("Invalid target ID: " + targetId + ". Must be 1-15.");
            }
            return;
        }

        switch (objectType) {
            case "OBJECT1":
                object1TargetId = targetId;
                break;
            case "OBJECT2":
                object2TargetId = targetId;
                break;
            case "OBJECT3":
                object3TargetId = targetId;
                break;
            case "OBJECT4":
                object4TargetId = targetId;
                break;
        }

        updateTargetDisplay(objectType, targetId);

        if (listener != null) {
            listener.onTargetUpdated(objectType, targetId);
            listener.onStatusMessage("Object " + objectType + " target ID updated to: " + targetId);
        }
    }

    public int getObjectTargetId(String objectType) {
        switch (objectType) {
            case "OBJECT1":
                return object1TargetId;
            case "OBJECT2":
                return object2TargetId;
            case "OBJECT3":
                return object3TargetId;
            case "OBJECT4":
                return object4TargetId;
            default:
                return 1;
        }
    }

    // Position Getters
    public int getObjectX(String objectType) {
        switch (objectType) {
            case "OBJECT1":
                return object1X;
            case "OBJECT2":
                return object2X;
            case "OBJECT3":
                return object3X;
            case "OBJECT4":
                return object4X;
            default:
                return -1;
        }
    }

    public int getObjectY(String objectType) {
        switch (objectType) {
            case "OBJECT1":
                return object1Y;
            case "OBJECT2":
                return object2Y;
            case "OBJECT3":
                return object3Y;
            case "OBJECT4":
                return object4Y;
            default:
                return -1;
        }
    }

    public boolean isObjectPlaced(String objectType) {
        switch (objectType) {
            case "OBJECT1":
                return object1Placed;
            case "OBJECT2":
                return object2Placed;
            case "OBJECT3":
                return object3Placed;
            case "OBJECT4":
                return object4Placed;
            default:
                return false;
        }
    }

    // Visual Updates
    private void updateDirectionIndicator(String objectType, String direction) {
        View directionIndicator = getDirectionView(objectType);
        if (directionIndicator == null) return;

        FrameLayout.LayoutParams params = (FrameLayout.LayoutParams) directionIndicator.getLayoutParams();

        switch (direction) {
            case "N": // North - horizontal line at top
                params.width = dpToPx(20);
                params.height = dpToPx(3);
                params.gravity = android.view.Gravity.TOP | android.view.Gravity.CENTER_HORIZONTAL;
                params.setMargins(0, dpToPx(2), 0, 0);
                break;

            case "S": // South - horizontal line at bottom
                params.width = dpToPx(20);
                params.height = dpToPx(3);
                params.gravity = android.view.Gravity.BOTTOM | android.view.Gravity.CENTER_HORIZONTAL;
                params.setMargins(0, 0, 0, dpToPx(2));
                break;

            case "E": // East - vertical line at right
                params.width = dpToPx(3);
                params.height = dpToPx(20);
                params.gravity = android.view.Gravity.RIGHT | android.view.Gravity.CENTER_VERTICAL;
                params.setMargins(0, 0, dpToPx(2), 0);
                break;

            case "W": // West - vertical line at left
                params.width = dpToPx(3);
                params.height = dpToPx(20);
                params.gravity = android.view.Gravity.LEFT | android.view.Gravity.CENTER_VERTICAL;
                params.setMargins(dpToPx(2), 0, 0, 0);
                break;
        }

        directionIndicator.setLayoutParams(params);
    }

    private void updateTargetDisplay(String objectType, int targetId) {
        TextView textView = getTextView(objectType);
        if (textView != null) {
            textView.setText(String.valueOf(targetId));
            textView.setTextSize(android.util.TypedValue.COMPLEX_UNIT_SP, 14);
        }
    }

    private void initializeDirectionIndicators() {
        updateDirectionIndicator("OBJECT1", "N");
        updateDirectionIndicator("OBJECT2", "N");
        updateDirectionIndicator("OBJECT3", "N");
        updateDirectionIndicator("OBJECT4", "N");
    }

    private void updateAllTargetDisplays() {
        // Initialize all objects with default size 7sp and their default numbers
        initializeTargetDisplay("OBJECT1", object1TargetId);
        initializeTargetDisplay("OBJECT2", object2TargetId);
        initializeTargetDisplay("OBJECT3", object3TargetId);
        initializeTargetDisplay("OBJECT4", object4TargetId);
    }

    private void initializeTargetDisplay(String objectType, int targetId) {
        TextView textView = getTextView(objectType);
        if (textView != null) {
            textView.setText(String.valueOf(targetId));
            textView.setTextSize(android.util.TypedValue.COMPLEX_UNIT_SP, 7);
        }
    }

    // Helper Methods
    private View getDirectionView(String objectType) {
        switch (objectType) {
            case "OBJECT1":
                return object1DirectionView;
            case "OBJECT2":
                return object2DirectionView;
            case "OBJECT3":
                return object3DirectionView;
            case "OBJECT4":
                return object4DirectionView;
            default:
                return null;
        }
    }

    private TextView getTextView(String objectType) {
        switch (objectType) {
            case "OBJECT1":
                return object1TextView;
            case "OBJECT2":
                return object2TextView;
            case "OBJECT3":
                return object3TextView;
            case "OBJECT4":
                return object4TextView;
            default:
                return null;
        }
    }

    private int dpToPx(int dp) {
        float density = context.getResources().getDisplayMetrics().density;
        return Math.round(dp * density);
    }

    // Status Methods
    public String getObjectStatus(String objectType) {
        int x = getObjectX(objectType);
        int y = getObjectY(objectType);
        String direction = getObjectDirection(objectType);
        int targetId = getObjectTargetId(objectType);

        if (isObjectPlaced(objectType)) {
            return objectType + " Target#" + targetId + ": (" + x + "," + y + "," + direction + ")";
        } else {
            return objectType + " Target#" + targetId + ": Not placed (" + direction + ")";
        }
    }

    public String getAllObjectsStatus() {
        StringBuilder status = new StringBuilder();
        status.append("Objects Status:\n");
        status.append(getObjectStatus("OBJECT1")).append("\n");
        status.append(getObjectStatus("OBJECT2")).append("\n");
        status.append(getObjectStatus("OBJECT3")).append("\n");
        status.append(getObjectStatus("OBJECT4"));
        return status.toString();
    }
}