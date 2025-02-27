import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, PointCloud2, PointField
from std_msgs.msg import String
import os
import csv
import struct

class TripleTemperaturePlot(Node):
    def __init__(self):
        super().__init__('triple_temperature_plot')

        # ------------------- File / Log Setup -------------------
        self.log_filename = "plot_data_log.csv"
        self.all_data = []  # holds (time, temperature, command, is_first_point)

        # Load existing CSV to restore any previous "lifetime" data
        self._load_previous_data()

        # ------------------- ROS Setup -------------------
        self.temp_sub = self.create_subscription(
            Temperature,
            '/temperature',
            self._temp_callback,
            10
        )
        self.cmd_sub = self.create_subscription(
            String,
            '/command',
            self._cmd_callback,
            10
        )

        self.cloud_pub = self.create_publisher(PointCloud2, 'temperature_plot', 10)
        self.timer_ = self.create_timer(1.0, self._publish_callback)

        # Node start time (not strictly required but available)
        self.start_time = self.get_clock().now().nanoseconds * 1.0e-9

        # Current command, next reading is "first" after command change
        self.last_command = "off"
        self.awaiting_first_point = False

        # ------------------- Time Windows -------------------
        # 1) Short term: 30 minutes
        self.short_term_window = 30.0 * 60.0   # 1800 seconds
        # 2) Medium term: 6 hours
        self.mid_term_window = 6.0 * 3600.0    # 21600 seconds

        # ------------------- Bounding Boxes -------------------
        # Format: (xmin, xmax, ymin, ymax)
        # A) Short-term
        self.bbox_short = (-2.1, -0.1, -1.0,  1.0)
        # B) Medium-term
        self.bbox_mid   = ( 0.1,  2.1, -1.0,  1.0)
        # C) Lifetime
        self.bbox_life  = (-2.1,  2.1, -1.7, -1.2)

        # Pre-create black line points for each box
        self.bbox_packed_short = self._create_bbox_line_points(*self.bbox_short)
        self.bbox_packed_mid   = self._create_bbox_line_points(*self.bbox_mid)
        self.bbox_packed_life  = self._create_bbox_line_points(*self.bbox_life)

        self.get_logger().info("TripleTemperaturePlot node started.")

    # ------------------- Data Loading / Saving -------------------
    def _load_previous_data(self):
        """Load any existing CSV with: time, temperature, command, is_first(0/1)."""
        if not os.path.exists(self.log_filename):
            return

        try:
            with open(self.log_filename, 'r') as f:
                reader = csv.reader(f)
                count = 0
                for row in reader:
                    if len(row) != 4:
                        continue
                    t_str, temp_str, cmd_str, first_str = row
                    t_val = float(t_str)
                    temp_val = float(temp_str)
                    cmd_val = cmd_str.strip()
                    first_val = (first_str.strip() == '1')
                    self.all_data.append((t_val, temp_val, cmd_val, first_val))
                    count += 1
            self.get_logger().info(f"Loaded {count} data rows from {self.log_filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to load {self.log_filename}: {e}")

    def _append_to_log_file(self, t_val, temperature, command, is_first):
        """
        Append a single line to CSV: time, temperature, command, is_first(0/1).
        """
        try:
            with open(self.log_filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    f"{t_val}",
                    f"{temperature}",
                    command,
                    '1' if is_first else '0'
                ])
        except Exception as e:
            self.get_logger().error(f"Could not append to {self.log_filename}: {e}")

    # ------------------- Subscription Callbacks -------------------
    def _cmd_callback(self, msg):
        """
        Latch the command ("on"/"off") => next Temperature is flagged as 'first_after_cmd'.
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
        On each Temperature, record (time, temp, command, is_first) and append to CSV.
        """
        t_now = self.get_clock().now().nanoseconds * 1.0e-9
        is_first = self.awaiting_first_point
        if is_first:
            self.awaiting_first_point = False

        self.all_data.append((t_now, msg.temperature, self.last_command, is_first))
        self._append_to_log_file(t_now, msg.temperature, self.last_command, is_first)

        self.get_logger().info(
            f"Received Temp={msg.temperature:.2f}, cmd='{self.last_command}', "
            f"{'(first_after_cmd)' if is_first else ''}"
        )

    # ------------------- Main Publish Timer -------------------
    def _publish_callback(self):
        """
        Publish a PointCloud2 with:
          - short-term data (last 30min)
          - medium-term data (last 6hr)
          - lifetime data (all)
        in three bounding boxes, using fixed padding in both x and y (0.1).
        """
        if not self.all_data:
            return

        now_sec = self.get_clock().now().nanoseconds * 1.0e-9

        # 1) short-term data
        st_data = [
            (t, temp, cmd, first)
            for (t, temp, cmd, first) in self.all_data
            if t >= now_sec - self.short_term_window
        ]
        # 2) medium-term data
        mt_data = [
            (t, temp, cmd, first)
            for (t, temp, cmd, first) in self.all_data
            if t >= now_sec - self.mid_term_window
        ]
        # 3) lifetime data
        lt_data = self.all_data

        if not (st_data or mt_data or lt_data):
            return

        # Min/max for each subset
        def get_min_max(data_list):
            if not data_list:
                return (0.0, 1.0, 0.0, 1.0)
            t_vals = [d[0] for d in data_list]
            temp_vals = [d[1] for d in data_list]
            return (min(t_vals), max(t_vals), min(temp_vals), max(temp_vals))

        st_tmin, st_tmax, st_temp_min, st_temp_max = get_min_max(st_data)
        mt_tmin, mt_tmax, mt_temp_min, mt_temp_max = get_min_max(mt_data)
        lt_tmin, lt_tmax, lt_temp_min, lt_temp_max = get_min_max(lt_data)

        # Start building the point list
        packed_points = []
        # Add bounding-box lines
        packed_points.extend(self.bbox_packed_short)
        packed_points.extend(self.bbox_packed_mid)
        packed_points.extend(self.bbox_packed_life)

        # --------------- Fixed 0.1 Padding in x & y ---------------
        def pad_dim(low, high, pad=0.1):
            """
            Shift boundary by pad on each side.
            If the dimension < 2*pad, fallback to a minimal gap of 0.01 each side.
            """
            dim = high - low
            if dim <= 2*pad:
                # fallback
                return (low + 0.01, high - 0.01)
            return (low + pad, high - pad)

        # short box
        st_xmin, st_xmax, st_ymin, st_ymax = self.bbox_short
        st_xmin_eff, st_xmax_eff = pad_dim(st_xmin, st_xmax, 0.1)
        st_ymin_eff, st_ymax_eff = pad_dim(st_ymin, st_ymax, 0.1)

        # medium box
        mt_xmin, mt_xmax, mt_ymin, mt_ymax = self.bbox_mid
        mt_xmin_eff, mt_xmax_eff = pad_dim(mt_xmin, mt_xmax, 0.1)
        mt_ymin_eff, mt_ymax_eff = pad_dim(mt_ymin, mt_ymax, 0.1)

        # lifetime box
        lf_xmin, lf_xmax, lf_ymin, lf_ymax = self.bbox_life
        lf_xmin_eff, lf_xmax_eff = pad_dim(lf_xmin, lf_xmax, 0.1)
        lf_ymin_eff, lf_ymax_eff = pad_dim(lf_ymin, lf_ymax, 0.1)

        # ---------- short-term data ----------
        for (t, temp, cmd, first_pt) in st_data:
            x = self._linear_map(t, st_tmin, st_tmax, st_xmin_eff, st_xmax_eff)
            y = self._linear_map(temp, st_temp_min, st_temp_max, st_ymin_eff, st_ymax_eff)
            z = 0.01 if first_pt else 0.0
            rgb = self._decide_rgb(temp, st_temp_min, st_temp_max, cmd, first_pt)
            packed_points.append(struct.pack('ffff', x, y, z, rgb))

        # ---------- medium-term data ----------
        for (t, temp, cmd, first_pt) in mt_data:
            x = self._linear_map(t, mt_tmin, mt_tmax, mt_xmin_eff, mt_xmax_eff)
            y = self._linear_map(temp, mt_temp_min, mt_temp_max, mt_ymin_eff, mt_ymax_eff)
            z = 0.01 if first_pt else 0.0
            rgb = self._decide_rgb(temp, mt_temp_min, mt_temp_max, cmd, first_pt)
            packed_points.append(struct.pack('ffff', x, y, z, rgb))

        # ---------- lifetime data ----------
        for (t, temp, cmd, first_pt) in lt_data:
            x = self._linear_map(t, lt_tmin, lt_tmax, lf_xmin_eff, lf_xmax_eff)
            y = self._linear_map(temp, lt_temp_min, lt_temp_max, lf_ymin_eff, lf_ymax_eff)
            z = 0.01 if first_pt else 0.0
            rgb = self._decide_rgb(temp, lt_temp_min, lt_temp_max, cmd, first_pt)
            packed_points.append(struct.pack('ffff', x, y, z, rgb))

        # Build and publish the PointCloud2
        pc_msg = self._create_pointcloud2(packed_points)
        self.cloud_pub.publish(pc_msg)
        self.get_logger().info(
            f"Published pointcloud: short={len(st_data)} pts, "
            f"mid={len(mt_data)} pts, life={len(lt_data)} pts"
        )

    # ------------------- Helper Methods -------------------
    def _decide_rgb(self, temp, tmin, tmax, cmd, is_first):
        """
        Return a float 'rgb':
          - If first after cmd => green if cmd='on', black if cmd='off'
          - Else => blue->red gradient based on temp
        """
        if is_first:
            if cmd == "on":
                return self._pack_rgb_to_float(0, 255, 0)  # green
            else:
                return self._pack_rgb_to_float(0, 0, 0)    # black

        if abs(tmax - tmin) < 1e-9:
            frac = 0.5
        else:
            frac = (temp - tmin) / (tmax - tmin)
            frac = max(0.0, min(1.0, frac))
        # linear interpolation: blue(0,0,255)->red(255,0,0)
        r = int(frac * 255)
        g = 0
        b = int((1.0 - frac) * 255)
        return self._pack_rgb_to_float(r, g, b)

    def _pack_rgb_to_float(self, r, g, b):
        """Convert (r,g,b) in [0..255] to a single float 'rgb' (PCL style)."""
        rgb_int = (r << 16) | (g << 8) | b
        return struct.unpack('!f', struct.pack('!I', rgb_int))[0]

    def _linear_map(self, val, in_min, in_max, out_min, out_max):
        """Map val from [in_min..in_max] to [out_min..out_max], handle degenerate range."""
        if abs(in_max - in_min) < 1e-9:
            return 0.5*(out_min + out_max)
        ratio = (val - in_min) / (in_max - in_min)
        return out_min + ratio * (out_max - out_min)

    def _create_pointcloud2(self, packed_points):
        """
        Return a PointCloud2 with (x,y,z,rgb) as float32.
        That way RViz can display color with 'RGB8'.
        """
        pc_msg = PointCloud2()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = 'map'

        pc_msg.height = 1
        pc_msg.width = len(packed_points)

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

    def _create_bbox_line_points(self, xmin, xmax, ymin, ymax):
        """
        Return black line points around the bounding box edges,
        with horizontal and vertical sampling proportional to dimension.
        """
        black_rgb = self._pack_rgb_to_float(0, 0, 0)

        width = abs(xmax - xmin)
        height = abs(ymax - ymin)

        steps_x = max(2, int(width * 50))
        steps_y = max(2, int(height * 50))

        def interpolate_segment(x1, y1, x2, y2, num_steps):
            seg_points = []
            for i in range(num_steps+1):
                frac = i / float(num_steps)
                x = x1 + frac*(x2 - x1)
                y = y1 + frac*(y2 - y1)
                z = 0.0
                seg_points.append(struct.pack('ffff', x, y, z, black_rgb))
            return seg_points

        pts = []
        # bottom edge
        pts += interpolate_segment(xmin, ymin, xmax, ymin, steps_x)
        # right edge
        pts += interpolate_segment(xmax, ymin, xmax, ymax, steps_y)
        # top edge
        pts += interpolate_segment(xmax, ymax, xmin, ymax, steps_x)
        # left edge
        pts += interpolate_segment(xmin, ymax, xmin, ymin, steps_y)

        return pts


def main(args=None):
    rclpy.init(args=args)
    node = TripleTemperaturePlot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
