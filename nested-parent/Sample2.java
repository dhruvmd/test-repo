import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;

public class SelfDrivingCar extends JPanel implements ActionListener {

    // --- Simulation Parameters ---
    private final int WIDTH = 800;
    private final int HEIGHT = 600;
    private final int DELAY = 20; // milliseconds
    private Timer timer;

    // --- Car State ---
    private double carX = 100;
    private double carY = HEIGHT / 2;
    private double carAngle = 0; // degrees
    private double carSpeed = 0;
    private final double MAX_SPEED = 5;
    private final double ACCELERATION = 0.1;
    private final double BRAKE_DECELERATION = 0.2;
    private final double STEERING_SENSITIVITY = 1.5; // degrees per frame

    // --- Sensor Data (Simulated) ---
    private List<Double> lidarReadings = new ArrayList<>();
    private double cameraLeftDistance = Double.MAX_VALUE;
    private double cameraRightDistance = Double.MAX_VALUE;

    // --- Road Data (Simplified) ---
    private List<Point2D.Double> roadPoints = new ArrayList<>();

    // --- Control Parameters ---
    private double targetSpeed = 3;
    private double laneCenterOffset = 0;
    private double steeringAngle = 0;

    // --- Flags ---
    private boolean accelerating = false;
    private boolean braking = false;
    private boolean steeringLeft = false;
    private boolean steeringRight = false;

    public SelfDrivingCar() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.DARK_GRAY);
        setFocusable(true);
        requestFocusInWindow();

        // Initialize Road (Simple straight line with some noise)
        for (int x = 0; x < WIDTH * 2; x += 10) {
            double y = HEIGHT / 2 + Math.sin(x * 0.01) * 30 + (Math.random() - 0.5) * 20;
            roadPoints.add(new Point2D.Double(x, y));
        }

        // Initialize Timer
        timer = new Timer(DELAY, this);
        timer.start();

        // Key Listener
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = true;
                if (key == KeyEvent.VK_DOWN) braking = true;
                if (key == KeyEvent.VK_LEFT) steeringLeft = true;
                if (key == KeyEvent.VK_RIGHT) steeringRight = true;
            }

            @Override
            public void keyReleased(KeyEvent e) {
                int key = e.getKeyCode();
                if (key == KeyEvent.VK_UP) accelerating = false;
                if (key == KeyEvent.VK_DOWN) braking = false;
                if (key == KeyEvent.VK_LEFT) steeringLeft = false;
                if (key == KeyEvent.VK_RIGHT) steeringRight = false;
            }
        });
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        updateCarState();
        updateSensors();
        updateControl();
        repaint();
    }

    private void updateCarState() {
        // --- Acceleration/Deceleration ---
        if (accelerating) {
            carSpeed = Math.min(carSpeed + ACCELERATION, MAX_SPEED);
        } else if (braking) {
            carSpeed = Math.max(carSpeed - BRAKE_DECELERATION, 0);
        } else {
            // Natural deceleration
            carSpeed = Math.max(carSpeed - 0.02, 0);
        }

        // --- Steering ---
        if (steeringLeft) {
            steeringAngle = Math.max(steeringAngle - STEERING_SENSITIVITY, -30);
        } else if (steeringRight) {
            steeringAngle = Math.min(steeringAngle + STEERING_SENSITIVITY, 30);
        } else {
            // Return to center steering
            if (steeringAngle > 0) steeringAngle = Math.max(steeringAngle - 1, 0);
            else if (steeringAngle < 0) steeringAngle = Math.min(steeringAngle + 1, 0);
        }

        carAngle += steeringAngle * (carSpeed / MAX_SPEED); // Steering effect scales with speed

        // --- Movement ---
        double angleRad = Math.toRadians(carAngle);
        carX += carSpeed * Math.cos(angleRad);
        carY += carSpeed * Math.sin(angleRad);

        // --- Boundary Conditions (Simple wrap-around) ---
        if (carX > WIDTH) carX = 0;
        if (carX < 0) carX = WIDTH;
        if (carY > HEIGHT) carY = 0;
        if (carY < 0) carY = HEIGHT;
    }

    private void updateSensors() {
        // --- Lidar Simulation (Simplified) ---
        lidarReadings.clear();
        int numRays = 36; // One ray every 10 degrees
        for (int i = 0; i < numRays; i++) {
            double rayAngle = i * 10; // Degrees
            double distance = simulateLidarRay(rayAngle);
            lidarReadings.add(distance);
        }

        // --- Camera Simulation (Simplified - just distance to lane markers) ---
        cameraLeftDistance = Double.MAX_VALUE;
        cameraRightDistance = Double.MAX_VALUE;

        // Find closest road points to the left and right of the car
        double carLeftX = carX - 20; //  "Left" camera position
        double carRightX = carX + 20; // "Right" camera position

        for (Point2D.Double roadPoint : roadPoints) {
            double distance = roadPoint.distance(carLeftX, carY);
            if (roadPoint.x < carX && distance < cameraLeftDistance) {
                cameraLeftDistance = distance;
            }
            distance = roadPoint.distance(carRightX, carY);
            if (roadPoint.x > carX && distance < cameraRightDistance) {
                cameraRightDistance = distance;
            }
        }
    }

    private double simulateLidarRay(double rayAngleDegrees) {
        double rayAngleRadians = Math.toRadians(carAngle + rayAngleDegrees);
        double rayX = carX;
        double rayY = carY;
        double distance = 0;
        double stepSize = 5;

        while (distance < 200) { // Max lidar range
            rayX += stepSize * Math.cos(rayAngleRadians);
            rayY += stepSize * Math.sin(rayAngleRadians);
            distance += stepSize;

            // Check for intersection with road
            for (int i = 0; i < roadPoints.size() - 1; i++) {
                Point2D.Double p1 = roadPoints.get(i);
                Point2D.Double p2 = roadPoints.get(i + 1);
                if (intersects(carX, carY, rayX, rayY, p1.x, p1.y, p2.x, p2.y)) {
                    return distance;
                }
            }

            // Check for out-of-bounds
            if (rayX < 0 || rayX > WIDTH * 2 || rayY < 0 || rayY > HEIGHT) {
                return Double.MAX_VALUE; // No hit
            }
        }
        return Double.MAX_VALUE; // No hit within range
    }

    // Helper function for line intersection (from internet)
    private boolean intersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (det == 0) {
            return false;
        }
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private void updateControl() {
        // --- Speed Control (Simple proportional control) ---
        double speedError = targetSpeed - carSpeed;
        if (speedError > 0) accelerating = true; else accelerating = false;
        if (speedError < 0) braking = true; else braking = false;

        // --- Lane Keeping (Proportional control based on camera data) ---
        laneCenterOffset = cameraLeftDistance - cameraRightDistance; // Positive = car is too far right
        double steeringCorrection = laneCenterOffset * 0.1; // Adjust gain as needed
        steeringAngle = steeringCorrection;
        steeringAngle = Math.max(Math.min(steeringAngle, 30), -30); // Clamp steering angle
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // --- Draw Road ---
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < roadPoints.size() - 1; i++) {
            Point2D.Double p1 = roadPoints.get(i);
            Point2D.Double p2 = roadPoints.get(i + 1);
            g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }

        // --- Draw Car ---
        AffineTransform originalTransform = g2d.getTransform();
        g2d.translate(carX, carY);
        g2d.rotate(Math.toRadians(carAngle));

        g2d.setColor(Color.RED);
        g2d.fillRect(-15, -10, 30, 20); // Car body
        g2d.setColor(Color.BLACK);
        g2d.fillRect(10, -8, 5, 16); // Spoiler

        g2d.setTransform(originalTransform); // Restore original transform

        // --- Draw Lidar Rays ---
        g2d.setColor(Color.YELLOW);
        for (int i = 0; i < lidarReadings.size(); i++) {
            double distance = lidarReadings.get(i);
            if (distance != Double.MAX_VALUE) {
                double rayAngle = Math.toRadians(carAngle + i * 10);
                double endX = carX + distance * Math.cos(rayAngle);
                double endY = carY + distance * Math.sin(rayAngle);
                g2d.drawLine((int) carX, (int) carY, (int) endX, (int) endY);
            }
        }

        // --- Draw Camera Ranges ---
        g2d.setColor(Color.GREEN);
        double leftCamX = carX - 20;
        double rightCamX = carX + 20;
        if (cameraLeftDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)leftCamX, (int)carY, (int)(leftCamX - cameraLeftDistance), (int)carY);
        }
         if (cameraRightDistance != Double.MAX_VALUE) {
             g2d.drawLine((int)rightCamX, (int)carY, (int)(rightCamX + cameraRightDistance), (int)carY);
        }


        // --- Draw Debug Information ---
        g2d.setColor(Color.WHITE);
        g2d.drawString("Speed: " + String.format("%.2f", carSpeed), 10, 20);
        g2d.drawString("Angle: " + String.format("%.2f", carAngle), 10, 40);
        g2d.drawString("Steering: " + String.format("%.2f", steeringAngle), 10, 60);
        g2d.drawString("Lane Offset: " + String.format("%.2f", laneCenterOffset), 10, 80);
        g2d.drawString("Camera Left: " + String.format("%.2f", cameraLeftDistance), 10, 100);
        g2d.drawString("Camera Right: " + String.format("%.2f", cameraRightDistance), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Self-Driving Car Simulation");
        SelfDrivingCar carPanel = new SelfDrivingCar();
        frame.add(carPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}

