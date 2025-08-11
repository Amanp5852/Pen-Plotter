**CoreXY ESP32 Pen Plotter**
This project is a fully functional CoreXY-based pen plotter designed for precision and reliability, combining mechanical engineering, electronics, and embedded programming into a single cohesive system.

The motion system is driven by GT2 timing belts and precision drivers, ensuring smooth and accurate X/Y movement. The CoreXY configuration provides efficient motion control with reduced inertia, allowing for higher speed and better precision compared to traditional Cartesian setups. The entire motion control is powered by an ESP32 microcontroller, enabling both high-speed processing and integrated connectivity options.

A key highlight of this system is its PID-based closed-loop control. Using feedback from optical encoders, the plotter continuously monitors its position and corrects any deviation in real-time. This ensures that plotted paths are highly accurate and consistent, even during complex or long-duration drawing tasks.

For the Z-axis, a servo motor is used to actuate the pen holder, allowing precise up and down movement for clean and controlled plotting. This makes it possible to handle intricate patterns and line transitions smoothly.

The plotter also integrates limit switches for homing operations, providing a predefined and repeatable reference position at startup. In addition, an emergency stop button is included, allowing the user to instantly halt all motion and safely return the plotter to the home position. This feature is especially important for preventing damage to the machine or workpiece in case of unexpected issues.

Mechanical components such as the frame, carriage, belt tensioners, and pen holder are custom 3D-printed for a lightweight yet rigid structure. This allows for easy customization and replacement of parts, making the system both versatile and user-friendly.

Overall, this project demonstrates a seamless integration of mechanical design, precise motion control, and robust safety features. It serves as both a learning platform for motion control systems and a functional tool for producing high-quality plotted artwork or technical drawings.
