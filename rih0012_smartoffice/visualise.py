import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, PointCloud2, PointField
from std_msgs.msg import String
import struct

class DualTemperaturePlot(Node):
    def __init__(self):
        super().__init__('dual_temperature_plot')

        # Subscriptions
        self.temp_sub = self.create_subscription(
            Temperature,
            '/temperature',  # adjust if needed
            self._temp_callback,
            10
        )
        self.cmd_sub = self.create_subscription(
            String,
            '/command',      # adjust if needed
            self._cmd_callback,
            10
        )

        # Publisher
        self.cloud_pub = self.create_publisher(PointCloud2, 'temperature_plot', 10)

        # Publish timer (1 Hz)
        self.timer_ = self.create_timer(1.0, self._publish_callback)

        # Track the node start time (for long‐term data)
        self.start_time = self.get_clock().now().nanoseconds * 1.0e-9

        # Store all temperature readings:
        # (time, temperature, command_string, is_first_after_command)
        self.all_data = []

        # Latch the last command (default off).
        # Also track if the next temperature reading is the first after a command.
        self.last_command = "off"
        self.awaiting_first_point = False

        # 30 min short‐term window (in seconds)
        self.short_term_window = 1800.0

        # Define bounding boxes for short vs. long data
        # Format: (xmin, xmax, ymin, ymax)
        self.bbox_short = (-2.1, -0.1, -1.0, 1.0)
        self.bbox_long  = ( 0.1,  2.1, -1.0, 1.0)

        # Pre‐create black line points for each bounding box
        self.bbox_packed_points_short = self._create_bbox_line_points(*self.bbox_short)
        self.bbox_packed_points_long  = self._create_bbox_line_points(*self.bbox_long)

        self.get_logger().info("DualTemperaturePlot node started.")

    def _cmd_callback(self, msg):
        """
        Latch the command ("on"/"off") and mark that the next temperature reading
        is the first after the command changed (for special coloring).
        """
        cmd = msg.data.strip().lower()
        if cmd in ("on", "off"):
            self.last_command = cmd
            self.awaiting_first_point = True
            self.get_logger().info(f"Command latched as '{cmd}'")
        else:
            self.get_logger().warn(f"Unknown command '{cmd}' (ignored).")

    def _temp_callback(self, msg):
        """
        On each Temperature, store: (time, temperature, latched_command, is_first_point).
        Then reset `awaiting_first_point` if needed.
        """
        t_now = self.get_clock().now().nanoseconds * 1.0e-9
        is_first = self.awaiting_first_point
        if is_first:
            self.awaiting_first_point = False

        self.all_data.append((t_now, msg.temperature, self.last_command, is_first))
        self.get_logger().info(
            f"Received Temperature={msg.temperature:.2f}, cmd='{self.last_command}', "
            f"{'(first_after_cmd)' if is_first else ''}"
        )

    def _publish_callback(self):
        """
        Publish a PointCloud2 containing:
          - black bounding box lines for short‐ and long‐term boxes
          - short‐term data (last 30 min) mapped into left box
          - long‐term data (all since start) mapped into right box
          - color by temperature (blue→red), except the first point after /command:
            => black if command=off, green if command=on
          - if 'is_first_point' => elevate z by 0.01 for clarity
        """
        if not self.all_data:
            return

        now_sec = self.get_clock().now().nanoseconds * 1.0e-9

        # 1) Split short‐term data vs. long‐term
        st_data = [
            (t, temp, cmd, first) 
            for (t, temp, cmd, first) in self.all_data
            if t >= (now_sec - self.short_term_window)
        ]
        lt_data = self.all_data  # everything

        if not st_data and not lt_data:
            return

        # 2) Determine the min/max time & temperature in each data set
        def get_min_max(data_list):
            # returns (time_min, time_max, temp_min, temp_max)
            if not data_list:
                return (0.0, 1.0, 0.0, 1.0)
            t_vals = [d[0] for d in data_list]
            temp_vals = [d[1] for d in data_list]
            return (min(t_vals), max(t_vals), min(temp_vals), max(temp_vals))

        st_tmin, st_tmax, st_temp_min, st_temp_max = get_min_max(st_data)
        lt_tmin, lt_tmax, lt_temp_min, lt_temp_max = get_min_max(lt_data)

        # 3) Build the final list of points
        packed_points = []
        packed_points.extend(self.bbox_packed_points_short)
        packed_points.extend(self.bbox_packed_points_long)

        # bounding boxes with 5% padding
        st_xmin, st_xmax, st_ymin, st_ymax = self.bbox_short
        st_xmin_eff, st_xmax_eff = self._pad_range(st_xmin, st_xmax, 0.05)
        st_ymin_eff, st_ymax_eff = self._pad_range(st_ymin, st_ymax, 0.05)

        lt_xmin, lt_xmax, lt_ymin, lt_ymax = self.bbox_long
        lt_xmin_eff, lt_xmax_eff = self._pad_range(lt_xmin, lt_xmax, 0.05)
        lt_ymin_eff, lt_ymax_eff = self._pad_range(lt_ymin, lt_ymax, 0.05)

        # 4) Add points for short‐term data
        for (t, temp, cmd, first_pt) in st_data:
            x = self._linear_map(t, st_tmin, st_tmax, st_xmin_eff, st_xmax_eff)
            y = self._linear_map(temp, st_temp_min, st_temp_max, st_ymin_eff, st_ymax_eff)
            # If this is the first point after a command, elevate it slightly
            z = 0.01 if first_pt else 0.0

            color = self._decide_color(temp, st_temp_min, st_temp_max, cmd, first_pt)
            rgb_float = self._pack_rgb_to_float(*color)

            point_bytes = struct.pack('ffff', x, y, z, rgb_float)
            packed_points.append(point_bytes)

        # 5) Add points for long‐term data
        for (t, temp, cmd, first_pt) in lt_data:
            x = self._linear_map(t, lt_tmin, lt_tmax, lt_xmin_eff, lt_xmax_eff)
            y = self._linear_map(temp, lt_temp_min, lt_temp_max, lt_ymin_eff, lt_ymax_eff)
            z = 0.01 if first_pt else 0.0  # same logic

            color = self._decide_color(temp, lt_temp_min, lt_temp_max, cmd, first_pt)
            rgb_float = self._pack_rgb_to_float(*color)

            point_bytes = struct.pack('ffff', x, y, z, rgb_float)
            packed_points.append(point_bytes)

        # 6) Build and publish the PointCloud2
        pc_msg = self._create_pointcloud2(packed_points)
        self.cloud_pub.publish(pc_msg)
        self.get_logger().info(
            f"Published pointcloud: short={len(st_data)} pts, long={len(lt_data)} pts"
        )

    # -----------------------------------------------
    # Helper methods
    # -----------------------------------------------
    def _decide_color(self, temp, tmin, tmax, cmd, is_first_point):
        """
        Decide the (r,g,b) color for a given reading.
         - If this reading is the *first* after a command:
             => black if cmd='off', green if cmd='on'
         - Otherwise, color by temperature from blue(0,0,255) to red(255,0,0).
        """
        if is_first_point:
            if cmd == "on":
                return (0, 255, 0)   # green
            else:
                return (0, 0, 0)     # black

        # Otherwise, do a blue→red gradient
        if abs(tmax - tmin) < 1e-9:
            frac = 0.5
        else:
            frac = (temp - tmin) / (tmax - tmin)
            frac = max(0.0, min(1.0, frac))

        # Interpolate from (0,0,255) -> (255,0,0)
        r = int(frac * 255)
        g = 0
        b = int((1.0 - frac) * 255)
        return (r, g, b)

    def _pack_rgb_to_float(self, r, g, b):
        """
        Pack three 8-bit channels (r,g,b) into a single float 'rgb'
        as done in typical PCL / RViz usage:
          uint32_t rgb = ( (r << 16) | (g << 8) | (b) );
          float rgb_float = reinterpret_cast<float&>(rgb);
        """
        rgb_int = (r << 16) | (g << 8) | (b)
        return struct.unpack('!f', struct.pack('!I', rgb_int))[0]

    def _linear_map(self, val, in_min, in_max, out_min, out_max):
        """Safely map val from [in_min, in_max] to [out_min, out_max]."""
        if abs(in_max - in_min) < 1e-9:
            return 0.5 * (out_min + out_max)
        ratio = (val - in_min) / (in_max - in_min)
        return out_min + ratio * (out_max - out_min)

    def _pad_range(self, mn, mx, frac=0.05):
        """
        Returns a slightly shrunken [mn..mx] so the data doesn't
        exactly touch bounding box edges (5% by default).
        """
        if mx <= mn:
            return (mn, mx)
        span = mx - mn
        offset = span * frac
        return (mn + offset, mx - offset)

    def _create_pointcloud2(self, packed_points):
        """
        Build a PointCloud2 with:
          x, y, z, rgb (each float32)
        so that RViz can use the 'RGB' transformer.
        """
        pc_msg = PointCloud2()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = 'map'

        pc_msg.height = 1
        pc_msg.width = len(packed_points)

        # We have 4 float32 => x,y,z,rgb => 16 bytes total
        pc_msg.fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.is_bigendian = False
        pc_msg.point_step = 16
        pc_msg.row_step = pc_msg.point_step * pc_msg.width
        pc_msg.data = b''.join(packed_points)
        pc_msg.is_dense = True

        return pc_msg

    def _create_bbox_line_points(self, xmin, xmax, ymin, ymax, steps=50):
        """
        Return points for lines around the bounding box edges, in black.
        Each point: (x, y, z, rgbfloat).
        """
        black_rgb_float = self._pack_rgb_to_float(0, 0, 0)
        def segment(x1, y1, x2, y2):
            seg_points = []
            for i in range(steps+1):
                frac = i / float(steps)
                x = x1 + frac*(x2 - x1)
                y = y1 + frac*(y2 - y1)
                z = 0.0
                p_bytes = struct.pack('ffff', x, y, z, black_rgb_float)
                seg_points.append(p_bytes)
            return seg_points

        pts = []
        # bottom edge
        pts += segment(xmin, ymin, xmax, ymin)
        # right edge
        pts += segment(xmax, ymin, xmax, ymax)
        # top edge
        pts += segment(xmax, ymax, xmin, ymax)
        # left edge
        pts += segment(xmin, ymax, xmin, ymin)

        return pts


def main(args=None):
    rclpy.init(args=args)
    node = DualTemperaturePlot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
