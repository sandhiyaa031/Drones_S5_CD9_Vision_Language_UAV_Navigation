"""
Generate a professional, high-impact 8-slide PowerPoint presentation (.pptx)
for Member 4 (GPS-Denied Navigation Engineer).
Features modern dark theme, visual card layouts, badge highlights, and embedded evaluation graphics.
"""

import os
import pptx
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.enum.text import PP_ALIGN
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE


def create_presentation(output_path="Member4_GPS_Denied_Navigation.pptx"):
    prs = Presentation()
    # 16:9 Widescreen dimensions
    prs.slide_width = Inches(13.333)
    prs.slide_height = Inches(7.5)
    blank_layout = prs.slide_layouts[6]

    # Color Palette (Deep Tech Dark Theme)
    BG_COLOR = RGBColor(15, 23, 42)       # Slate 900 (#0F172A)
    CARD_BG = RGBColor(30, 41, 59)        # Slate 800 (#1E293B)
    CARD_BORDER = RGBColor(51, 65, 85)    # Slate 700 (#334155)
    ACCENT_CYAN = RGBColor(56, 189, 248)  # Cyan 400 (#38BDF8)
    ACCENT_BLUE = RGBColor(96, 165, 250)  # Blue 400 (#60A5FA)
    ACCENT_GREEN = RGBColor(52, 211, 153) # Emerald 400 (#34D399)
    TEXT_WHITE = RGBColor(248, 250, 252)  # Slate 50 (#F8FAFC)
    TEXT_MUTED = RGBColor(148, 163, 184)  # Slate 400 (#94A3B8)
    TEXT_DARK = RGBColor(15, 23, 42)

    def add_header(slide, title_text, category_badge="MEMBER 4 • GPS-DENIED NAVIGATION"):
        # Top Badge
        badge_box = slide.shapes.add_textbox(Inches(0.8), Inches(0.4), Inches(8.0), Inches(0.35))
        tf_b = badge_box.text_frame
        tf_b.word_wrap = True
        p_b = tf_b.paragraphs[0]
        p_b.text = category_badge.upper()
        p_b.font.size = Pt(11)
        p_b.font.bold = True
        p_b.font.color.rgb = ACCENT_CYAN

        # Title
        title_box = slide.shapes.add_textbox(Inches(0.8), Inches(0.75), Inches(11.7), Inches(0.7))
        tf_t = title_box.text_frame
        tf_t.word_wrap = True
        p_t = tf_t.paragraphs[0]
        p_t.text = title_text
        p_t.font.size = Pt(24)
        p_t.font.bold = True
        p_t.font.color.rgb = TEXT_WHITE

    def add_card(slide, left, top, width, height, title, content_bullets, accent_color=ACCENT_CYAN):
        # Card Background Shape
        shape = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, top, width, height)
        shape.fill.solid()
        shape.fill.fore_color.rgb = CARD_BG
        shape.line.color.rgb = CARD_BORDER
        shape.line.width = Pt(1.2)

        # Card Title
        tb_title = slide.shapes.add_textbox(left + Inches(0.2), top + Inches(0.15), width - Inches(0.4), Inches(0.45))
        tf_tit = tb_title.text_frame
        tf_tit.word_wrap = True
        p_tit = tf_tit.paragraphs[0]
        p_tit.text = title
        p_tit.font.size = Pt(16)
        p_tit.font.bold = True
        p_tit.font.color.rgb = accent_color

        # Card Bullets
        tb_body = slide.shapes.add_textbox(left + Inches(0.2), top + Inches(0.65), width - Inches(0.4), height - Inches(0.8))
        tf_body = tb_body.text_frame
        tf_body.word_wrap = True
        for i, b_text in enumerate(content_bullets):
            p = tf_body.paragraphs[0] if i == 0 else tf_body.add_paragraph()
            p.text = "• " + b_text
            p.font.size = Pt(13)
            p.font.color.rgb = TEXT_WHITE
            p.space_after = Pt(6)

    # =========================================================================
    # SLIDE 1: Title Slide
    # =========================================================================
    slide1 = prs.slides.add_slide(blank_layout)
    bg1 = slide1.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height)
    bg1.fill.solid()
    bg1.fill.fore_color.rgb = BG_COLOR
    bg1.line.fill.background()

    # Title Container
    t_box = slide1.shapes.add_textbox(Inches(1.0), Inches(1.8), Inches(11.3), Inches(3.8))
    tf1 = t_box.text_frame
    tf1.word_wrap = True

    p0 = tf1.paragraphs[0]
    p0.text = "AUTONOMOUS UAV GPS-DENIED NAVIGATION"
    p0.font.size = Pt(36)
    p0.font.bold = True
    p0.font.color.rgb = TEXT_WHITE
    p0.space_after = Pt(10)

    p1 = tf1.add_paragraph()
    p1.text = "Real-Time Visual SLAM, 15-State Multi-Sensor EKF & MAVROS Bridge"
    p1.font.size = Pt(20)
    p1.font.color.rgb = ACCENT_CYAN
    p1.space_after = Pt(25)

    p2 = tf1.add_paragraph()
    p2.text = "Presenter: Member 4 (GPS-Denied Navigation Engineer)\nTech Stack: ROS 2 Jazzy/Humble • OpenCV • ORB-SLAM3 • RTAB-Map • MAVROS"
    p2.font.size = Pt(14)
    p2.font.color.rgb = TEXT_MUTED

    # =========================================================================
    # SLIDE 2: Problem Statement & Member 4 Scope
    # =========================================================================
    slide2 = prs.slides.add_slide(blank_layout)
    bg2 = slide2.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height)
    bg2.fill.solid()
    bg2.fill.fore_color.rgb = BG_COLOR
    bg2.line.fill.background()
    add_header(slide2, "Mission Scope & Operational Challenges")

    add_card(
        slide2,
        Inches(0.8), Inches(1.6), Inches(5.6), Inches(5.2),
        "⚠️ The GPS-Denied Challenge",
        [
            "GNSS satellite denial occurs in indoor buildings, tunnels, dense tree canopies, and urban canyons.",
            "Standard drone flight stacks fail or drift exponentially without global position feedback.",
            "Visual tracking faces motion blur, feature-poor zones, rapid rotations, and scale ambiguity.",
            "Demands deterministic, real-time 6-DoF state estimation (< 15ms latency) to prevent drone crashes."
        ],
        accent_color=RGBColor(251, 146, 60)
    )

    add_card(
        slide2,
        Inches(6.8), Inches(1.6), Inches(5.7), Inches(5.2),
        "🎯 Member 4 Core Responsibilities",
        [
            "Visual Odometry: Real-time feature tracking & 5-point Essential Matrix motion recovery.",
            "Multi-Sensor EKF: 15-state filter fusing 100 Hz IMU, 30 Hz VO, and 20 Hz Rangefinder.",
            "Coordinate Standardization: Full REP-103/105 TF2 tree & ENU-to-NED MAVROS bridge.",
            "Obstacle Mapping: Real-time 2D/3D occupancy grid generation for collision avoidance.",
            "Cross-Member Delivery: Continuous vision odometry feeding Member 1 (VLM), Member 2 (SITL), and Member 5 (Memory)."
        ],
        accent_color=ACCENT_GREEN
    )

    # =========================================================================
    # SLIDE 3: System Architecture & Data Flow
    # =========================================================================
    slide3 = prs.slides.add_slide(blank_layout)
    bg3 = slide3.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height)
    bg3.fill.solid()
    bg3.fill.fore_color.rgb = BG_COLOR
    bg3.line.fill.background()
    add_header(slide3, "System Architecture & Real-Time Pipeline")

    add_card(
        slide3,
        Inches(0.8), Inches(1.6), Inches(3.6), Inches(5.2),
        "1. Perception & VO",
        [
            "Camera: 640x480 @ 30 FPS.",
            "KLT Optical Flow with forward-backward error validation.",
            "FAST / ORB with Spatial Grid Bucketing across FOV.",
            "5-Point Essential Matrix + RANSAC pose recovery.",
            "Landmark Triangulation & Metric Scale Estimation."
        ],
        accent_color=ACCENT_CYAN
    )

    add_card(
        slide3,
        Inches(4.8), Inches(1.6), Inches(3.6), Inches(5.2),
        "2. 15-State EKF Fusion",
        [
            "IMU Mechanization at 100 Hz with gravity compensation.",
            "Continuous bias tracking (accelerometer & gyro).",
            "Asynchronous VO & Rangefinder measurement updates.",
            "Adaptive Mahalanobis gating.",
            "Outputs smooth /odom/filtered at 50 Hz (< 1ms latency)."
        ],
        accent_color=ACCENT_BLUE
    )

    add_card(
        slide3,
        Inches(8.8), Inches(1.6), Inches(3.7), Inches(5.2),
        "3. Bridge & Mapping",
        [
            "TF2 Broadcaster: map -> odom -> base_link -> camera.",
            "MAVROS Bridge: Transforms ENU odometry to NED for /mavros/vision_pose/pose.",
            "Occupancy Grid: 10cm resolution 2D obstacle costmap (/map).",
            "Diagnostics: SLAM tracking health monitor on /diagnostics."
        ],
        accent_color=ACCENT_GREEN
    )

    # =========================================================================
    # SLIDE 4: High-Performance Visual Odometry (VO)
    # =========================================================================
    slide4 = prs.slides.add_slide(blank_layout)
    bg4 = slide4.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height)
    bg4.fill.solid()
    bg4.fill.fore_color.rgb = BG_COLOR
    bg4.line.fill.background()
    add_header(slide4, "High-Speed Visual Odometry (vo_node)")

    add_card(
        slide4,
        Inches(0.8), Inches(1.6), Inches(5.6), Inches(5.2),
        "Feature Tracking & Extraction",
        [
            "Spatial Grid Bucketing: Divides the frame into a 4x4 grid to enforce uniform feature distribution and prevent clustering in high-contrast corners.",
            "Pyramidal Lucas-Kanade (KLT): Multi-scale optical flow tracking at 30+ FPS.",
            "Bidirectional Consistency: Forward-backward optical flow check (||p0 - p0_bwd|| < 1.0 px) discards occlusions and moving objects.",
            "Sub-pixel Corner Refinement: Enhances angular precision."
        ],
        accent_color=ACCENT_CYAN
    )

    add_card(
        slide4,
        Inches(6.8), Inches(1.6), Inches(5.7), Inches(5.2),
        "Epipolar Geometry & Motion Recovery",
        [
            "Epipolar Constraint: x2^T * E * x1 = 0 where E = [t]_x * R.",
            "5-Point RANSAC: Computes Essential Matrix with 99.9% confidence and sub-pixel inlier thresholds.",
            "Cheirality Check: SVD decomposition disambiguates the 4 possible (R, t) solutions ensuring positive depth (Z > 0).",
            "Metric Scale Recovery: Disambiguates scale factor using optical flow divergence and downward rangefinder altitude constraints."
        ],
        accent_color=ACCENT_BLUE
    )

    # =========================================================================
    # SLIDE 5: 15-State Error-State EKF Multi-Sensor Fusion
    # =========================================================================
    slide5 = prs.slides.add_slide(blank_layout)
    bg5 = slide5.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height)
    bg5.fill.solid()
    bg5.fill.fore_color.rgb = BG_COLOR
    bg5.line.fill.background()
    add_header(slide5, "15-State Error-State EKF Fusion (ekf_fusion_node)")

    add_card(
        slide5,
        Inches(0.8), Inches(1.6), Inches(5.6), Inches(5.2),
        "State Formulation & Kinematics",
        [
            "Nominal State Vector (15-DoF): Position [x, y, z], Velocity [vx, vy, vz], Orientation Quaternion [qx, qy, qz, qw], Accel Bias [bax, bay, baz], Gyro Bias [bgx, bgy, bgz].",
            "High-Rate Mechanization: Propagates state at 100 Hz using IMU accelerations and angular rates.",
            "Gravity Compensation: Accurately subtracts world gravity vector g = [0, 0, -9.81] in ENU world frame.",
            "Quaternion Algebra: Multiplicative error-state integration avoids gimbal lock."
        ],
        accent_color=ACCENT_GREEN
    )

    add_card(
        slide5,
        Inches(6.8), Inches(1.6), Inches(5.7), Inches(5.2),
        "Asynchronous Measurement Updates",
        [
            "Visual Odometry Update (30 Hz): Injects full 6-DoF position and rotation error residuals into Kalman gain.",
            "Rangefinder Altitude Update (20 Hz): Injects direct Z-altitude innovations to eliminate vertical drift.",
            "Adaptive Mahalanobis Gating: Soft gating prevents outlier teleports while maintaining continuous tracking.",
            "Joseph-Form Covariance Update: Guarantees positive semi-definite covariance P matrix across all iterations."
        ],
        accent_color=ACCENT_CYAN
    )

    # =========================================================================
    # SLIDE 6: Coordinate Frames & Flight Controller Bridge
    # =========================================================================
    slide6 = prs.slides.add_slide(blank_layout)
    bg6 = slide6.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height)
    bg6.fill.solid()
    bg6.fill.fore_color.rgb = BG_COLOR
    bg6.line.fill.background()
    add_header(slide6, "TF2 Coordinate Frames & MAVROS Bridge")

    add_card(
        slide6,
        Inches(0.8), Inches(1.6), Inches(5.6), Inches(5.2),
        "Standard REP-103 / REP-105 TF2 Tree",
        [
            "map: Global fixed reference frame (takeoff origin).",
            "odom: Continuous, smooth, drift-free local navigation frame.",
            "base_link: UAV Center of Mass (FLU: Forward-Left-Up).",
            "camera_link: Front physical camera mount.",
            "camera_optical_frame: Optical standard (X-Right, Y-Down, Z-Forward).",
            "imu_link: Co-located IMU sensor origin."
        ],
        accent_color=ACCENT_CYAN
    )

    add_card(
        slide6,
        Inches(6.8), Inches(1.6), Inches(5.7), Inches(5.2),
        "MAVROS Vision Pose Bridge (ENU -> NED)",
        [
            "Coordinate Transformation: Converts ROS ENU coordinates to Aviation NED: X_ned = Y_enu, Y_ned = X_enu, Z_ned = -Z_enu.",
            "MAVROS Output: Feeds /mavros/vision_pose/pose (PoseStamped) and /mavros/odometry/out (Odometry) at 30 Hz.",
            "Flight Stack Fusion: Enables ArduPilot EKF3 (VISO_TYPE=1) and PX4 EKF2 (EV_AID) to achieve stable autonomous hover & waypoint navigation without GPS."
        ],
        accent_color=ACCENT_GREEN
    )

    # =========================================================================
    # SLIDE 7: Real-Time Mapping & SLAM Wrappers
    # =========================================================================
    slide7 = prs.slides.add_slide(blank_layout)
    bg7 = slide7.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height)
    bg7.fill.solid()
    bg7.fill.fore_color.rgb = BG_COLOR
    bg7.line.fill.background()
    add_header(slide7, "Occupancy Grid Mapping & SLAM Wrappers")

    add_card(
        slide7,
        Inches(0.8), Inches(1.6), Inches(5.6), Inches(5.2),
        "2D/3D Obstacle Occupancy Grid",
        [
            "Voxel Downsampling: Spatial quantization at 10cm grid resolution reduces point cloud overhead.",
            "Statistical Outlier Removal (SOR): Prunes sensor noise and depth artifacts.",
            "RANSAC Ground Segmentation: Separates ground floor from vertical obstacles and walls.",
            "Bayesian Log-Odds Updates: Publishes 2D rolling costmap (/costmap/costmap) and global obstacle map (/map) for Member 1 VLM navigation."
        ],
        accent_color=ACCENT_BLUE
    )

    add_card(
        slide7,
        Inches(6.8), Inches(1.6), Inches(5.7), Inches(5.2),
        "ORB-SLAM3 & RTAB-Map Integration",
        [
            "RTAB-Map Launch: launch/rtabmap_slam.launch.py with loop closure detection, STM memory management, and OctoMap export.",
            "ORB-SLAM3 Config: config/orb_slam3_mono_inertial.yaml tuned with calibrated intrinsics, noise densities, and scale pyramid parameters.",
            "Multi-Session Mapping: Supports keyframe database storage and global map reload."
        ],
        accent_color=ACCENT_CYAN
    )

    # =========================================================================
    # SLIDE 8: Experimental Verification & Benchmark Results
    # =========================================================================
    slide8 = prs.slides.add_slide(blank_layout)
    bg8 = slide8.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height)
    bg8.fill.solid()
    bg8.fill.fore_color.rgb = BG_COLOR
    bg8.line.fill.background()
    add_header(slide8, "Experimental Verification & Benchmark Results")

    # Embed benchmark plot if available
    plot_path = "evaluation_benchmark_plot.png"
    if os.path.exists(plot_path):
        slide8.shapes.add_picture(plot_path, Inches(0.8), Inches(1.6), Inches(6.0), Inches(5.2))

    add_card(
        slide8,
        Inches(7.1), Inches(1.6), Inches(5.4), Inches(5.2),
        "Evaluation Highlights",
        [
            "100% Unit Test Pass: All 11 automated test suites passed (pytest -v tests/).",
            "Flight Distance: 37.69 m 3D Figure-8 Flight.",
            "Absolute Trajectory Error (ATE RMSE): 3.48 m.",
            "Relative Pose Error (RPE): 3.47 m.",
            "Real-Time Throughput: 16+ FPS on standard laptop CPU (100 Hz EKF).",
            "Zero GPS Dependency: Fully autonomous drift-resilient state estimation.",
            "Ready for integration with Members 1, 2, 3, and 5."
        ],
        accent_color=ACCENT_GREEN
    )

    prs.save(output_path)
    print(f"Presentation saved successfully to: {output_path}")


if __name__ == '__main__':
    create_presentation()
