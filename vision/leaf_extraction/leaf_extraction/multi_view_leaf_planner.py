#!/usr/bin/env python3

"""Fuse explicit RGB-D views and rank collision-aware leaf grasp poses."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from typing import List, Optional, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion, Vector3
import numpy as np
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
import rclpy
from rclpy.time import Time
from scipy.spatial import ConvexHull, cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray

from custom_interfaces.msg import LeafPoseArrays
from leaf_extraction.leaf_surface_projection import (
    ProjectionInput,
    leaf_collision_groups,
    plant_root_anchor,
    primitive_surface_distance,
    project_candidate_groups,
)


@dataclass
class LeafObservation:
    """One segmented leaf instance transformed into the planning frame."""

    points: np.ndarray
    confidence: float
    touches_border: bool
    camera_position: np.ndarray
    camera_quaternion: Optional[np.ndarray] = None
    frontier_point: Optional[np.ndarray] = None
    frontier_direction: Optional[np.ndarray] = None
    view_id: int = 0


@dataclass
class LeafTrack:
    """Cross-view collection of observations believed to be the same leaf."""

    leaf_id: int
    observations: List[LeafObservation] = field(default_factory=list)

    @property
    def centroid(self) -> np.ndarray:
        centroids = [
            np.median(item.points, axis=0)
            for item in self.observations
        ]
        return np.mean(centroids, axis=0)


@dataclass
class GraspCandidate:
    """A T-REX-inspired surface point before robot feasibility checking."""

    leaf_id: int
    point: np.ndarray
    normal: np.ndarray
    tangent: np.ndarray
    score: float
    geometric_score: float
    confidence: float
    view_count: int
    edge_margin: float
    local_leaf_width: float
    edge_margin_ratio: float
    clearance: float
    longitudinal_ratio: float
    projection_distance: float = 0.0
    collision_leaf_id: str = ''


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def normalized(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError('Cannot normalize a zero-length vector')
    return vector / norm


def quaternion_message(quaternion: Sequence[float]) -> Quaternion:
    return Quaternion(
        x=float(quaternion[0]),
        y=float(quaternion[1]),
        z=float(quaternion[2]),
        w=float(quaternion[3]),
    )


class MultiViewLeafPlanner(Node):
    """Fuse stationary leaf observations and publish ranked grasp poses."""

    def __init__(self) -> None:
        super().__init__('multi_view_leaf_planner')
        default_model = os.path.join(
            get_package_share_directory('leaf_extraction'),
            'segmentation_model',
            'leaf_sim_best.pt',
        )
        self.declare_parameter(
            'point_cloud_topic', '/camera/depth/color/points')
        self.declare_parameter('target_frame', 'base')
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('scene_scale', 1.0)
        self.declare_parameter('required_views', 3)
        self.declare_parameter('minimum_leaf_views', 1)
        self.declare_parameter('maximum_views', 5)
        self.declare_parameter('association_distance', 0.10)
        self.declare_parameter('voxel_size', 0.004)
        self.declare_parameter('candidate_count', 8)
        self.declare_parameter('per_leaf_candidate_count', 4)
        self.declare_parameter('minimum_projected_candidates', 6)
        self.declare_parameter('minimum_candidate_leaves', 2)
        self.declare_parameter('minimum_longitudinal_ratio', 0.30)
        self.declare_parameter(
            'preferred_longitudinal_start', 0.55)
        self.declare_parameter(
            'preferred_longitudinal_end', 0.85)
        self.declare_parameter(
            'maximum_projection_width_ratio', 1.25)
        self.declare_parameter('minimum_surface_views', 2)
        self.declare_parameter('surface_support_voxels', 2.5)
        self.declare_parameter('minimum_edge_margin_ratio', 0.15)
        self.declare_parameter('approach_distance', 0.07)
        self.declare_parameter('minimum_clearance', 0.025)
        self.declare_parameter('view_lateral_offset', 0.10)
        self.declare_parameter('view_radial_scale', 0.90)
        self.declare_parameter('view_height', 0.14)
        self.declare_parameter('overview_extent_scale', 1.60)
        self.declare_parameter('overview_minimum_span', 0.45)
        self.declare_parameter('overview_image_margin', 0.82)
        self.declare_parameter('overview_elevation_degrees', 65.0)
        self.declare_parameter('minimum_downward_pitch_degrees', 40.0)
        self.declare_parameter('maximum_downward_pitch_degrees', 90.0)
        self.declare_parameter('minimum_camera_above_canopy', 0.15)
        self.declare_parameter('camera_horizontal_fov', 1.047)
        self.declare_parameter('adaptive_ready_score', 0.62)
        self.declare_parameter('canopy_expansion_margin', 0.20)
        self.declare_parameter('initial_canopy_gate_margin_ratio', 0.25)
        self.declare_parameter('initial_canopy_gate_vertical_margin', 0.10)
        self.declare_parameter('minimum_view_coverage', 0.85)
        self.declare_parameter('view_frame_margin', 0.10)
        self.declare_parameter('low_surface_gain_ratio', 0.05)
        self.declare_parameter('low_surface_gain_patience', 2)
        self.declare_parameter('minimum_nbv_completion_gain', 0.08)
        self.declare_parameter(
            'minimum_nbv_angular_separation_degrees', 10.0)
        self.declare_parameter('minimum_nbv_translation', 0.04)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.scene_scale = float(self.get_parameter('scene_scale').value)
        self.metric_floor = 0.02 * self.scene_scale
        self.minimum_crop_height = 0.10 * self.scene_scale
        self.surface_noise_reference = 0.004 * self.scene_scale
        self.neighbour_distance_reference = 0.10 * self.scene_scale
        self.local_geometry_radius = 0.018 * self.scene_scale
        self.quality_radius_floor = 0.012 * self.scene_scale
        self.reachability_decay_length = 0.12 * self.scene_scale
        self.local_view_radius_gate = 0.02 * self.scene_scale
        self.local_view_planar_gate = 0.01 * self.scene_scale
        self.required_views = int(self.get_parameter('required_views').value)
        self.minimum_leaf_views = int(
            self.get_parameter('minimum_leaf_views').value)
        self.maximum_views = int(self.get_parameter('maximum_views').value)
        self.association_distance = self.scene_scale * float(
            self.get_parameter('association_distance').value)
        self.voxel_size = self.scene_scale * float(
            self.get_parameter('voxel_size').value)
        self.candidate_count = int(
            self.get_parameter('candidate_count').value)
        self.per_leaf_candidate_count = int(
            self.get_parameter('per_leaf_candidate_count').value)
        self.minimum_projected_candidates = int(
            self.get_parameter('minimum_projected_candidates').value)
        self.minimum_candidate_leaves = int(
            self.get_parameter('minimum_candidate_leaves').value)
        self.minimum_longitudinal_ratio = float(
            self.get_parameter('minimum_longitudinal_ratio').value)
        self.preferred_longitudinal_start = float(
            self.get_parameter('preferred_longitudinal_start').value)
        self.preferred_longitudinal_end = float(
            self.get_parameter('preferred_longitudinal_end').value)
        self.maximum_projection_width_ratio = float(
            self.get_parameter('maximum_projection_width_ratio').value)
        self.minimum_surface_views = int(
            self.get_parameter('minimum_surface_views').value)
        self.surface_support_voxels = float(
            self.get_parameter('surface_support_voxels').value)
        self.minimum_edge_margin_ratio = float(
            self.get_parameter('minimum_edge_margin_ratio').value)
        self.approach_distance = self.scene_scale * float(
            self.get_parameter('approach_distance').value)
        self.minimum_clearance = self.scene_scale * float(
            self.get_parameter('minimum_clearance').value)
        self.view_lateral_offset = self.scene_scale * float(
            self.get_parameter('view_lateral_offset').value)
        self.view_radial_scale = float(
            self.get_parameter('view_radial_scale').value)
        self.view_height = self.scene_scale * float(
            self.get_parameter('view_height').value)
        self.overview_extent_scale = float(
            self.get_parameter('overview_extent_scale').value)
        self.overview_minimum_span = self.scene_scale * float(
            self.get_parameter('overview_minimum_span').value)
        self.overview_image_margin = float(
            self.get_parameter('overview_image_margin').value)
        self.overview_elevation_degrees = float(
            self.get_parameter('overview_elevation_degrees').value)
        self.minimum_downward_pitch_degrees = float(
            self.get_parameter('minimum_downward_pitch_degrees').value)
        self.maximum_downward_pitch_degrees = float(
            self.get_parameter('maximum_downward_pitch_degrees').value)
        self.minimum_camera_above_canopy = self.scene_scale * float(
            self.get_parameter('minimum_camera_above_canopy').value)
        if not (
            0.0 < self.minimum_downward_pitch_degrees
            < self.maximum_downward_pitch_degrees <= 90.0
        ):
            raise ValueError(
                'Downward camera pitch limits must satisfy '
                '0 < minimum < maximum <= 90 degrees')
        self.camera_horizontal_fov = float(
            self.get_parameter('camera_horizontal_fov').value)
        self.adaptive_ready_score = float(
            self.get_parameter('adaptive_ready_score').value)
        self.canopy_expansion_margin = self.scene_scale * float(
            self.get_parameter('canopy_expansion_margin').value)
        self.initial_canopy_gate_margin_ratio = float(
            self.get_parameter('initial_canopy_gate_margin_ratio').value)
        self.initial_canopy_gate_vertical_margin = self.scene_scale * float(
            self.get_parameter(
                'initial_canopy_gate_vertical_margin').value)
        self.minimum_view_coverage = float(
            self.get_parameter('minimum_view_coverage').value)
        self.view_frame_margin = float(
            self.get_parameter('view_frame_margin').value)
        self.low_surface_gain_ratio = float(
            self.get_parameter('low_surface_gain_ratio').value)
        self.low_surface_gain_patience = int(
            self.get_parameter('low_surface_gain_patience').value)
        self.minimum_nbv_completion_gain = float(
            self.get_parameter('minimum_nbv_completion_gain').value)
        self.minimum_nbv_angular_separation = math.radians(float(
            self.get_parameter(
                'minimum_nbv_angular_separation_degrees').value))
        self.minimum_nbv_translation = self.scene_scale * float(
            self.get_parameter('minimum_nbv_translation').value)
        self.confidence = float(self.get_parameter('confidence').value)

        model_path = str(self.get_parameter('model_path').value)
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f'Leaf segmentation model does not exist: {model_path}')
        self.model = YOLO(model_path)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_cloud: Optional[PointCloud2] = None
        self.tracks: List[LeafTrack] = []
        self.view_count = 0
        self.next_leaf_id = 1
        self.selection_ready = False
        self.coarse_canopy_center: Optional[np.ndarray] = None
        self.coarse_canopy_span = 0.0
        self.conservative_lower: Optional[np.ndarray] = None
        self.conservative_upper: Optional[np.ndarray] = None
        self.initial_gate_lower: Optional[np.ndarray] = None
        self.initial_gate_upper: Optional[np.ndarray] = None
        self.previous_surface_voxel_keys = set()
        self.low_surface_gain_streak = 0
        self.last_best_completion_gain = 0.0

        durable = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        point_cloud_topic = str(
            self.get_parameter('point_cloud_topic').value)
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            point_cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.capture_service = self.create_service(
            Trigger, '/leaf_perception/capture_view', self._capture_view)
        self.overview_service = self.create_service(
            Trigger,
            '/leaf_perception/prepare_overview',
            self._prepare_overview,
        )
        self.reset_service = self.create_service(
            Trigger, '/leaf_perception/reset_views', self._reset_views)
        self.finalize_service = self.create_service(
            Trigger, '/leaf_perception/finalize', self._finalize)
        self.bounds_publisher = self.create_publisher(
            Marker, '/leaf_perception/canopy_bounds', durable)
        self.views_publisher = self.create_publisher(
            PoseArray, '/leaf_perception/observation_views', durable)
        self.candidates_publisher = self.create_publisher(
            LeafPoseArrays, '/target_leaves_multi_pose', durable)
        self.markers_publisher = self.create_publisher(
            MarkerArray,
            '/leaf_perception/projected_grasp_candidates',
            durable,
        )
        self.get_logger().info(
            'Multi-view leaf planner ready; move to a stable observation pose '
            'and call /leaf_perception/capture_view')

    def _cloud_callback(self, message: PointCloud2) -> None:
        self.latest_cloud = message

    def _prepare_overview(self, _request, response):
        """Estimate a conservative full-plant view from the safe pose."""
        if self.latest_cloud is None:
            response.success = False
            response.message = 'No organized RGB point cloud has arrived'
            return response
        try:
            observations, transform = self._segment_cloud(self.latest_cloud)
        except (ValueError, TransformException) as exc:
            response.success = False
            response.message = str(exc)
            return response
        if not observations:
            response.success = False
            response.message = (
                'No leaf foreground was available for overview estimation')
            return response

        points = np.concatenate(
            [item.points for item in observations], axis=0)
        lower = np.percentile(points, 2.0, axis=0)
        upper = np.percentile(points, 98.0, axis=0)
        detected_size = np.maximum(
            upper - lower, 0.02 * self.scene_scale)
        conservative_size = detected_size.copy()
        conservative_size[:2] = np.maximum(
            self.overview_extent_scale * detected_size[:2],
            self.overview_minimum_span,
        )
        conservative_size[2] = max(
            (1.0 + self.canopy_expansion_margin) * detected_size[2],
            0.25 * self.overview_minimum_span,
        )
        self.coarse_canopy_center = 0.5 * (lower + upper)
        self.conservative_lower = (
            self.coarse_canopy_center - 0.5 * conservative_size)
        self.conservative_upper = (
            self.coarse_canopy_center + 0.5 * conservative_size)
        gate_margin = np.array([
            self.initial_canopy_gate_margin_ratio * conservative_size[0],
            self.initial_canopy_gate_margin_ratio * conservative_size[1],
            max(
                self.initial_canopy_gate_vertical_margin,
                self.initial_canopy_gate_margin_ratio
                * conservative_size[2],
            ),
        ])
        self.initial_gate_lower = self.conservative_lower - gate_margin
        self.initial_gate_upper = self.conservative_upper + gate_margin
        self.coarse_canopy_span = float(max(conservative_size[:2]))
        count, retreat, centring, best_coverage = (
            self._publish_overview_views(
                points, self.latest_cloud, transform)
        )
        response.success = count > 0
        response.message = (
            f'overview_candidates={count}; '
            f'observed_bounds=({lower[0]:.3f},{lower[1]:.3f},'
            f'{lower[2]:.3f})-({upper[0]:.3f},{upper[1]:.3f},'
            f'{upper[2]:.3f}); '
            f'conservative_span={self.coarse_canopy_span:.3f} m; '
            f'preferred_overview_elevation='
            f'{self.overview_elevation_degrees:.1f} deg; '
            f'downward_pitch_range='
            f'{self.minimum_downward_pitch_degrees:.1f}-'
            f'{self.maximum_downward_pitch_degrees:.1f} deg; '
            f'best_predicted_coverage={best_coverage:.3f}; '
            f'maximum_candidate_motion={retreat:.3f} m; '
            f'centring_shift={centring:.3f} m')
        return response

    def _reset_views(self, _request, response):
        self.tracks.clear()
        self.view_count = 0
        self.next_leaf_id = 1
        self.selection_ready = False
        self.coarse_canopy_center = None
        self.coarse_canopy_span = 0.0
        self.conservative_lower = None
        self.conservative_upper = None
        self.initial_gate_lower = None
        self.initial_gate_upper = None
        self.previous_surface_voxel_keys = set()
        self.low_surface_gain_streak = 0
        self.last_best_completion_gain = 0.0
        response.success = True
        response.message = 'Cleared all multi-view observations'
        return response

    def _capture_view(self, _request, response):
        if self.latest_cloud is None:
            response.success = False
            response.message = 'No organized RGB point cloud has arrived'
            return response
        try:
            observations, transform = self._segment_cloud(self.latest_cloud)
        except (ValueError, TransformException) as exc:
            response.success = False
            response.message = str(exc)
            return response
        if not observations:
            response.success = False
            response.message = 'No valid leaf instances in this view'
            return response

        self._associate_observations(observations)
        self.view_count += 1
        all_points = self._fused_canopy_points()
        self._update_conservative_bounds(all_points)
        self._publish_canopy_bounds(all_points, self.latest_cloud)
        self.last_best_completion_gain = self._publish_observation_views(
            all_points, self.latest_cloud)

        voxel_keys = np.unique(
            np.floor(all_points / self.voxel_size).astype(np.int64),
            axis=0,
        )
        current_keys = {tuple(key) for key in voxel_keys}
        if self.previous_surface_voxel_keys:
            novel_keys = (
                current_keys - self.previous_surface_voxel_keys)
            surface_gain = len(novel_keys) / max(len(current_keys), 1)
            if surface_gain < self.low_surface_gain_ratio:
                self.low_surface_gain_streak += 1
            else:
                self.low_surface_gain_streak = 0
        else:
            surface_gain = 1.0
        self.previous_surface_voxel_keys.update(current_keys)

        candidates = self._rank_candidates()
        candidate_leaf_count = len({
            candidate.leaf_id for candidate in candidates})
        candidate_set_sufficient = (
            len(candidates) >= self.minimum_projected_candidates
            and candidate_leaf_count >= self.minimum_candidate_leaves
        )
        # The overview may create provisional candidates for directing NBV,
        # but a final MTC target always needs one independent validation view.
        enough_views = self.view_count >= 2
        lower = np.percentile(all_points, 2.0, axis=0)
        upper = np.percentile(all_points, 98.0, axis=0)
        canopy_centre = 0.5 * (lower + upper)
        canopy_span = max(
            float(max(upper[0] - lower[0], upper[1] - lower[1])),
            self.coarse_canopy_span,
        )
        unresolved_frontiers = len(self._unresolved_frontier_targets(
            canopy_centre, canopy_span))
        forced = self.view_count >= self.maximum_views
        completion_condition = bool(
            enough_views and candidate_set_sufficient)
        self.selection_ready = self._ready_with_candidates(
            candidates, completion_condition)
        if self.selection_ready and candidates:
            self._publish_candidates(candidates, self.latest_cloud)

        response.success = True
        response.message = (
            f'captured_view={self.view_count}; tracks={len(self.tracks)}; '
            f'projected_candidates={len(candidates)}; '
            f'candidate_leaves={candidate_leaf_count}; '
            f'sufficient={candidate_set_sufficient}; '
            f'unresolved_frontiers={unresolved_frontiers}; '
            f'novel_voxel_gain={surface_gain:.3f}; '
            f'next_completion_gain='
            f'{self.last_best_completion_gain:.3f}; '
            f'forced_limit={forced}; '
            f'ready={self.selection_ready}')
        return response

    @staticmethod
    def _ready_with_candidates(
        candidates: Sequence[GraspCandidate],
        completion_condition: bool,
    ) -> bool:
        """Never report completion before a grasp point can be published."""
        return bool(candidates) and bool(completion_condition)

    def _finalize(self, _request, response):
        candidates = self._rank_candidates()
        candidate_leaf_count = len({
            candidate.leaf_id for candidate in candidates})
        sufficient = (
            len(candidates) >= self.minimum_projected_candidates
            and candidate_leaf_count >= self.minimum_candidate_leaves
        )
        if self.view_count < 2 or not sufficient:
            response.success = False
            response.message = (
                f'Only {self.view_count} valid views, '
                f'{len(candidates)} projected candidates and '
                f'{candidate_leaf_count} candidate leaves are available; '
                f'need {self.minimum_projected_candidates} candidates '
                f'across {self.minimum_candidate_leaves} leaves')
            return response
        self._publish_candidates(candidates, self.latest_cloud)
        self.selection_ready = True
        response.success = True
        response.message = (
            f'Published {len(candidates)} candidates from '
            f'{self.view_count} valid views')
        return response

    def _segment_cloud(
        self, cloud: PointCloud2
    ) -> Tuple[List[LeafObservation], object]:
        if cloud.height <= 1 or cloud.width <= 1:
            raise ValueError('Point cloud must be organized')
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            cloud.header.frame_id,
            Time.from_msg(cloud.header.stamp),
            timeout=Duration(seconds=0.5),
        )
        points, colors = self._unpack_cloud(cloud)
        image = cv2.cvtColor(colors, cv2.COLOR_RGB2BGR)
        result = self.model.predict(
            image,
            imgsz=cloud.width,
            conf=self.confidence,
            verbose=False,
        )[0]
        if result.masks is None:
            return [], transform

        target_from_camera = Rotation.from_quat([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ])
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ])
        camera_position = translation.copy()
        observations = []
        confidences = result.boxes.conf.detach().cpu().numpy()
        for index, polygon in enumerate(result.masks.xy):
            polygon = np.asarray(polygon, dtype=np.int32)
            if polygon.shape[0] < 3:
                continue
            mask = np.zeros((cloud.height, cloud.width), dtype=np.uint8)
            cv2.fillPoly(mask, [polygon], 1)
            instance_mask = mask.astype(bool)
            instance_points = points[instance_mask]
            finite = np.all(np.isfinite(instance_points), axis=1)
            instance_points = instance_points[finite]
            instance_points = instance_points[
                instance_points[:, 2] > self.minimum_crop_height]
            if instance_points.shape[0] < 40:
                continue
            instance_points = self._robust_filter(instance_points)
            world_points = (
                target_from_camera.apply(instance_points) + translation)
            if (
                self.initial_gate_lower is not None
                and self.initial_gate_upper is not None
            ):
                in_initial_canopy = np.all(
                    (world_points >= self.initial_gate_lower)
                    & (world_points <= self.initial_gate_upper),
                    axis=1,
                )
                removed = int(np.count_nonzero(~in_initial_canopy))
                instance_points = instance_points[in_initial_canopy]
                world_points = world_points[in_initial_canopy]
                if removed:
                    self.get_logger().debug(
                        f'Rejected {removed} segmented depth points outside '
                        'the fixed initial full-plant envelope')
                if world_points.shape[0] < 40:
                    continue
            border_sides = (
                bool(np.any(polygon[:, 0] <= 2)),
                bool(np.any(polygon[:, 1] <= 2)),
                bool(np.any(polygon[:, 0] >= cloud.width - 3)),
                bool(np.any(polygon[:, 1] >= cloud.height - 3)),
            )
            touches_border = any(border_sides)
            frontier_point = None
            frontier_direction = None
            if touches_border:
                border_band = max(
                    4, int(round(0.02 * min(cloud.height, cloud.width))))
                frontier_mask = np.zeros_like(instance_mask)
                if border_sides[0]:
                    frontier_mask[:, :border_band] = True
                if border_sides[1]:
                    frontier_mask[:border_band, :] = True
                if border_sides[2]:
                    frontier_mask[:, cloud.width - border_band:] = True
                if border_sides[3]:
                    frontier_mask[cloud.height - border_band:, :] = True
                frontier_camera = points[instance_mask & frontier_mask]
                valid_frontier = (
                    np.all(np.isfinite(frontier_camera), axis=1)
                    & (frontier_camera[:, 2] > self.minimum_crop_height)
                )
                frontier_camera = frontier_camera[valid_frontier]
                if frontier_camera.shape[0] >= 6:
                    frontier_world = (
                        target_from_camera.apply(frontier_camera)
                        + translation
                    )
                    if (
                        self.initial_gate_lower is not None
                        and self.initial_gate_upper is not None
                    ):
                        valid_frontier_world = np.all(
                            (frontier_world >= self.initial_gate_lower)
                            & (frontier_world <= self.initial_gate_upper),
                            axis=1,
                        )
                        frontier_world = frontier_world[
                            valid_frontier_world]
                    if frontier_world.shape[0] >= 6:
                        leaf_centre = np.median(world_points, axis=0)
                        distances = np.linalg.norm(
                            frontier_world - leaf_centre, axis=1)
                        outer = frontier_world[
                            distances >= np.percentile(distances, 65.0)]
                        frontier_point = np.median(outer, axis=0)
                        outward = frontier_point - leaf_centre
                        if np.linalg.norm(outward) > self.voxel_size:
                            frontier_direction = normalized(outward)
            observations.append(LeafObservation(
                points=world_points,
                confidence=float(confidences[index]),
                touches_border=touches_border,
                camera_position=camera_position,
                camera_quaternion=target_from_camera.as_quat(),
                frontier_point=frontier_point,
                frontier_direction=frontier_direction,
            ))
        return observations, transform

    @staticmethod
    def _unpack_cloud(cloud: PointCloud2) -> Tuple[np.ndarray, np.ndarray]:
        fields = {field.name: field.offset for field in cloud.fields}
        if not all(name in fields for name in ('x', 'y', 'z', 'rgb')):
            raise ValueError('Point cloud must contain x, y, z and rgb fields')
        raw = np.frombuffer(cloud.data, dtype=np.uint8)
        count = cloud.height * cloud.width
        rows = raw[:count * cloud.point_step].reshape(count, cloud.point_step)
        points = np.column_stack([
            np.frombuffer(
                np.ascontiguousarray(
                    rows[:, fields[axis]:fields[axis] + 4]).tobytes(),
                dtype='>f4' if cloud.is_bigendian else '<f4',
            )
            for axis in ('x', 'y', 'z')
        ])
        packed = np.frombuffer(
            np.ascontiguousarray(
                rows[:, fields['rgb']:fields['rgb'] + 4]).tobytes(),
            dtype='>u4' if cloud.is_bigendian else '<u4',
        )
        colors = np.column_stack([
            (packed >> 16) & 0xFF,
            (packed >> 8) & 0xFF,
            packed & 0xFF,
        ]).astype(np.uint8)
        return (
            points.reshape(cloud.height, cloud.width, 3),
            colors.reshape(cloud.height, cloud.width, 3),
        )

    @staticmethod
    def _robust_filter(points: np.ndarray) -> np.ndarray:
        median = np.median(points, axis=0)
        absolute = np.abs(points - median)
        mad = np.median(absolute, axis=0)
        mad[mad < 1e-5] = 1e-5
        keep = np.all(absolute / (1.4826 * mad) < 3.5, axis=1)
        filtered = points[keep]
        return filtered if filtered.shape[0] >= 30 else points

    def _associate_observations(
        self, observations: Sequence[LeafObservation]
    ) -> None:
        available = set(range(len(self.tracks)))
        overlap_limit = max(
            3.0 * self.voxel_size,
            0.15 * self.association_distance,
        )
        observations = sorted(
            observations,
            key=lambda item: float(np.linalg.norm(np.median(
                item.points, axis=0))),
        )
        for observation in observations:
            observation.view_id = self.view_count + 1
            centroid = np.median(observation.points, axis=0)
            best_index = None
            best_score = float('inf')
            observation_normal = self._association_normal(
                observation.points)
            for index in available:
                track = self.tracks[index]
                centroid_distance = float(
                    np.linalg.norm(centroid - track.centroid))
                track_points = np.concatenate(
                    [item.points for item in track.observations], axis=0)
                overlap_distance = self._point_cloud_overlap_distance(
                    observation.points, track_points)
                track_normal = self._association_normal(track_points)
                normal_similarity = abs(float(np.dot(
                    observation_normal, track_normal)))
                geometry_matches = (
                    centroid_distance < self.association_distance
                    or overlap_distance < overlap_limit
                )
                if not geometry_matches or normal_similarity < 0.65:
                    continue
                score = min(
                    centroid_distance / self.association_distance,
                    overlap_distance / overlap_limit,
                ) + 0.25 * (1.0 - normal_similarity)
                if score < best_score:
                    best_score = score
                    best_index = index
            if best_index is None:
                self.tracks.append(LeafTrack(
                    leaf_id=self.next_leaf_id,
                    observations=[observation],
                ))
                self.next_leaf_id += 1
            else:
                self.tracks[best_index].observations.append(observation)
                available.remove(best_index)

    @staticmethod
    def _association_normal(points: np.ndarray) -> np.ndarray:
        centred = points - np.median(points, axis=0)
        covariance = centred.T @ centred / max(points.shape[0] - 1, 1)
        _, eigenvectors = np.linalg.eigh(covariance)
        return normalized(eigenvectors[:, 0])

    @staticmethod
    def _point_cloud_overlap_distance(
        first: np.ndarray, second: np.ndarray
    ) -> float:
        first_stride = max(1, first.shape[0] // 240)
        second_stride = max(1, second.shape[0] // 480)
        first_sample = first[::first_stride]
        second_sample = second[::second_stride]
        distances, _ = cKDTree(second_sample).query(
            first_sample, k=1)
        return float(np.percentile(distances, 10.0))

    def _voxel_fuse(
        self, observations: Sequence[LeafObservation]
    ) -> np.ndarray:
        points = np.concatenate([item.points for item in observations], axis=0)
        keys = np.floor(points / self.voxel_size).astype(np.int64)
        _, inverse = np.unique(keys, axis=0, return_inverse=True)
        sums = np.zeros((int(inverse.max()) + 1, 3), dtype=float)
        counts = np.bincount(inverse)
        np.add.at(sums, inverse, points)
        return sums / counts[:, None]

    def _fused_canopy_points(self) -> np.ndarray:
        """Return the latest fused canopy rather than only the newest frame."""
        fused = [
            self._voxel_fuse(track.observations)
            for track in self.tracks if track.observations
        ]
        return np.concatenate(fused, axis=0)

    def _consensus_surface_points(self, track: LeafTrack) -> np.ndarray:
        """Keep points supported by independent views on a real surface."""
        if not track.observations:
            return np.empty((0, 3))
        # The full-plant overview is allowed to propose provisional projected
        # candidates on its own.  As soon as NBV supplies a second view, go
        # back to the configured multi-view support threshold.
        required_support = min(
            self.minimum_surface_views, len(track.observations))
        fused = self._voxel_fuse(track.observations)
        support_radius = (
            self.surface_support_voxels * self.voxel_size)
        nearest_points = []
        supported = []
        for observation in track.observations:
            tree = cKDTree(observation.points)
            distances, indices = tree.query(fused)
            nearest_points.append(observation.points[indices])
            supported.append(distances <= support_radius)
        nearest_points = np.stack(nearest_points, axis=0)
        supported = np.stack(supported, axis=0)
        keep = np.count_nonzero(
            supported, axis=0) >= required_support

        surface_points = []
        for point_index in np.flatnonzero(keep):
            observations = nearest_points[
                supported[:, point_index], point_index, :]
            consensus = np.median(observations, axis=0)
            # Use the nearest actual observation rather than an interpolated
            # average, so the published marker remains on measured geometry.
            medoid_index = int(np.argmin(np.linalg.norm(
                observations - consensus, axis=1)))
            surface_points.append(observations[medoid_index])
        if not surface_points:
            return np.empty((0, 3))
        return self._voxel_fuse([
            LeafObservation(
                points=np.asarray(surface_points),
                confidence=1.0,
                touches_border=False,
                camera_position=np.zeros(3),
            )
        ])

    def _rank_candidates(self) -> List[GraspCandidate]:
        track_support = ','.join(
            f'{track.leaf_id}:{len(track.observations)}'
            for track in self.tracks
        )
        eligible = [
            track for track in self.tracks
            if len(track.observations) >= self.minimum_leaf_views
        ]
        if not eligible:
            self.get_logger().info(
                'Candidate audit: track_views=['
                f'{track_support}]; no multi-view leaf tracks')
            return []
        fused = {}
        consensus_counts = {}
        for track in eligible:
            consensus = self._consensus_surface_points(track)
            consensus_counts[track.leaf_id] = int(consensus.shape[0])
            if consensus.shape[0] >= 25:
                fused[track.leaf_id] = consensus
        eligible = [
            track for track in eligible if track.leaf_id in fused]
        if not eligible:
            self.get_logger().info(
                'Candidate audit: track_views=['
                f'{track_support}]; consensus={consensus_counts}; '
                'no track retained 25 surface-consensus points')
            return []
        per_leaf_candidates: List[List[GraspCandidate]] = []
        raw_candidate_counts = {}
        for track in eligible:
            points = fused[track.leaf_id]
            other_parts = [
                value for leaf_id, value in fused.items()
                if leaf_id != track.leaf_id
            ]
            other_points = (
                np.concatenate(other_parts, axis=0)
                if other_parts else np.empty((0, 3)))
            leaf_candidates = self._leaf_candidates(
                track, points, other_points)
            raw_candidate_counts[track.leaf_id] = len(leaf_candidates)
            if leaf_candidates:
                per_leaf_candidates.append(leaf_candidates)

        projection_inputs = []
        candidate_by_index = {}
        input_index = 0
        for leaf_candidates in per_leaf_candidates:
            for candidate in leaf_candidates:
                projection_inputs.append(ProjectionInput(
                    index=input_index,
                    perceived_leaf_id=candidate.leaf_id,
                    point=tuple(float(value) for value in candidate.point),
                    normal=tuple(float(value) for value in candidate.normal),
                    local_leaf_width=float(candidate.local_leaf_width),
                ))
                candidate_by_index[input_index] = candidate
                input_index += 1
        proxy_groups = leaf_collision_groups()
        proxy_gap_audit = {}
        for leaf_candidates in per_leaf_candidates:
            if not leaf_candidates:
                continue
            perceived_leaf_id = leaf_candidates[0].leaf_id
            proxy_gap_audit[perceived_leaf_id] = {
                collision_leaf_id: round(float(np.median([
                    min(
                        primitive_surface_distance(
                            candidate.point, collision_object)
                        for collision_object in collision_objects
                    )
                    for candidate in leaf_candidates
                ])), 4)
                for collision_leaf_id, collision_objects
                in proxy_groups.items()
            }
        projections, assignments = project_candidate_groups(
            projection_inputs,
            maximum_width_ratio=self.maximum_projection_width_ratio,
        )
        projected_per_leaf = {}
        for index, projection in projections.items():
            candidate = candidate_by_index[index]
            if (
                candidate.longitudinal_ratio
                < self.minimum_longitudinal_ratio
            ):
                continue
            candidate.point = np.asarray(projection.point)
            candidate.projection_distance = projection.distance
            candidate.collision_leaf_id = projection.collision_leaf_id
            projection_factor = math.exp(
                -projection.distance
                / max(candidate.local_leaf_width, self.voxel_size))
            candidate.score *= projection_factor
            projected_per_leaf.setdefault(
                candidate.leaf_id, []).append(candidate)
        per_leaf_candidates = []
        for leaf_id in sorted(projected_per_leaf):
            leaf_candidates = projected_per_leaf[leaf_id]
            leaf_candidates.sort(key=lambda item: item.score, reverse=True)
            per_leaf_candidates.append(
                leaf_candidates[:self.per_leaf_candidate_count])

        projected_counts = {
            leaf_id: len(items)
            for leaf_id, items in projected_per_leaf.items()
        }
        self.get_logger().info(
            'Candidate audit: '
            f'track_views=[{track_support}]; '
            f'consensus={consensus_counts}; '
            f'raw={raw_candidate_counts}; '
            f'proxy_median_gaps={proxy_gap_audit}; '
            f'projection_assignments={assignments}; '
            f'projected={projected_counts}')

        # Round-robin by within-leaf rank. This prevents one large, easy leaf
        # from consuming the entire global shortlist.
        candidates: List[GraspCandidate] = []
        for rank in range(self.per_leaf_candidate_count):
            same_rank = [
                leaf[rank] for leaf in per_leaf_candidates
                if rank < len(leaf)
            ]
            same_rank.sort(key=lambda item: item.score, reverse=True)
            candidates.extend(same_rank)
            if len(candidates) >= self.candidate_count:
                break
        return candidates[:self.candidate_count]

    @staticmethod
    def _local_leaf_width(
        point_2d: np.ndarray, polygon: np.ndarray
    ) -> float:
        """Measure leaf width at a point along its PCA long axis."""
        longitudinal = float(point_2d[0])
        intersections = []
        for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
            delta = end - start
            if abs(float(delta[0])) < 1e-9:
                if abs(longitudinal - float(start[0])) < 1e-9:
                    intersections.extend((float(start[1]), float(end[1])))
                continue
            fraction = (longitudinal - float(start[0])) / float(delta[0])
            if -1e-9 <= fraction <= 1.0 + 1e-9:
                intersections.append(
                    float(start[1] + fraction * delta[1]))
        if len(intersections) < 2:
            return 0.0
        return max(intersections) - min(intersections)

    def _leaf_candidates(
        self,
        track: LeafTrack,
        points: np.ndarray,
        other_points: np.ndarray,
    ) -> List[GraspCandidate]:
        if points.shape[0] < 25:
            return []
        centre = np.median(points, axis=0)
        covariance = np.cov((points - centre).T)
        values, vectors = np.linalg.eigh(covariance)
        order = np.argsort(values)
        normal = normalized(vectors[:, order[0]])
        if normal[2] < 0.0:
            normal = -normal
        tangent = normalized(vectors[:, order[-1]])
        bitangent = normalized(np.cross(normal, tangent))
        tangent = normalized(np.cross(bitangent, normal))
        projected = np.column_stack([
            (points - centre) @ tangent,
            (points - centre) @ bitangent,
        ])
        root_anchor = np.asarray(plant_root_anchor())
        longitudinal_min = float(np.min(projected[:, 0]))
        longitudinal_max = float(np.max(projected[:, 0]))
        minimum_endpoint = centre + longitudinal_min * tangent
        maximum_endpoint = centre + longitudinal_max * tangent
        if (
            np.linalg.norm(maximum_endpoint - root_anchor)
            < np.linalg.norm(minimum_endpoint - root_anchor)
        ):
            tangent = -tangent
            bitangent = normalized(np.cross(normal, tangent))
            tangent = normalized(np.cross(bitangent, normal))
            projected = np.column_stack([
                (points - centre) @ tangent,
                (points - centre) @ bitangent,
            ])
            longitudinal_min = float(np.min(projected[:, 0]))
            longitudinal_max = float(np.max(projected[:, 0]))
        longitudinal_span = max(
            longitudinal_max - longitudinal_min,
            3.0 * self.voxel_size,
        )
        try:
            hull = ConvexHull(projected)
        except Exception:
            return []
        polygon = projected[hull.vertices]
        polygon_mm = np.asarray(
            np.round(polygon * 1000.0), dtype=np.int32)
        leaf_tree = cKDTree(points)
        other_tree = cKDTree(other_points) if other_points.size else None
        view_support = clamp01(
            len(track.observations) / max(1, self.required_views))
        untruncated = np.mean([
            0.0 if item.touches_border else 1.0
            for item in track.observations
        ])
        detector_confidence = float(np.mean([
            item.confidence for item in track.observations]))
        surface_noise = math.sqrt(max(float(values[order[0]]), 0.0))
        normal_quality = math.exp(-surface_noise / max(
            self.surface_noise_reference, 1e-6))
        if other_tree is None:
            neighbour_score = 1.0
        else:
            neighbour_distance = float(other_tree.query(centre)[0])
            neighbour_score = clamp01(neighbour_distance / max(
                self.neighbour_distance_reference, 1e-6))
        visibility = clamp01(0.65 * view_support + 0.35 * untruncated)
        leaf_score = sum((
            0.35 * neighbour_score,
            0.30 * visibility,
            0.35 * detector_confidence,
        ))

        results = []
        sampling_step = max(1, points.shape[0] // 300)
        for index in range(0, points.shape[0], sampling_step):
            point = points[index]
            longitudinal_ratio = clamp01(
                (projected[index, 0] - longitudinal_min)
                / longitudinal_span)
            if longitudinal_ratio < self.minimum_longitudinal_ratio:
                continue
            point_2d = projected[index] * 1000.0
            edge_margin_mm = cv2.pointPolygonTest(
                polygon_mm,
                (float(point_2d[0]), float(point_2d[1])),
                True,
            )
            edge_margin = max(0.0, edge_margin_mm / 1000.0)
            local_leaf_width = self._local_leaf_width(
                projected[index], polygon)
            if local_leaf_width < 3.0 * self.voxel_size:
                continue
            edge_margin_ratio = edge_margin / max(
                0.5 * local_leaf_width, self.voxel_size)
            if edge_margin_ratio < self.minimum_edge_margin_ratio:
                continue
            neighbour_indices = leaf_tree.query_ball_point(
                point, self.local_geometry_radius)
            if len(neighbour_indices) < 8:
                continue
            local = points[neighbour_indices]
            local_values, local_vectors = np.linalg.eigh(
                np.cov((local - point).T))
            local_noise = math.sqrt(max(float(local_values[0]), 0.0))
            flatness = math.exp(-local_noise / max(
                0.0025 * self.scene_scale, 1e-6))
            local_normal = normalized(local_vectors[:, 0])
            if float(np.dot(local_normal, normal)) < 0.0:
                local_normal = -local_normal
            local_tangent = (
                tangent
                - float(np.dot(tangent, local_normal)) * local_normal
            )
            if float(np.linalg.norm(local_tangent)) < 1e-6:
                local_tangent = np.cross(bitangent, local_normal)
            local_tangent = normalized(local_tangent)
            alignment = abs(float(np.dot(
                local_normal, np.array([0.0, 0.0, 1.0]))))
            edge_score = clamp01(
                (edge_margin_ratio - self.minimum_edge_margin_ratio)
                / max(0.65 - self.minimum_edge_margin_ratio, 1e-6))
            clearance = self._approach_clearance(
                point, local_normal, local_tangent, other_tree)
            clearance_score = clamp01(
                clearance / max(self.minimum_clearance, 1e-6))
            if longitudinal_ratio < self.preferred_longitudinal_start:
                distal_score = clamp01(
                    (longitudinal_ratio - self.minimum_longitudinal_ratio)
                    / max(
                        self.preferred_longitudinal_start
                        - self.minimum_longitudinal_ratio,
                        1e-6,
                    ))
            elif longitudinal_ratio <= self.preferred_longitudinal_end:
                distal_score = 1.0
            else:
                distal_score = max(
                    0.55,
                    1.0
                    - 0.45
                    * (
                        longitudinal_ratio
                        - self.preferred_longitudinal_end
                    )
                    / max(
                        1.0 - self.preferred_longitudinal_end,
                        1e-6,
                    ),
                )
            geometry = sum((
                0.20 * flatness,
                0.20 * alignment,
                0.20 * edge_score,
                0.15 * clearance_score,
                0.25 * distal_score,
            ))
            confidence = clamp01(
                detector_confidence * view_support * normal_quality)
            # Preserve the multiplicative semantics while keeping one weak
            # term from collapsing every candidate to zero.
            score = np.prod((
                max(0.05, leaf_score),
                max(0.05, geometry),
                max(0.05, confidence),
            )) ** (1.0 / 3.0)
            results.append(GraspCandidate(
                leaf_id=track.leaf_id,
                point=point,
                normal=local_normal,
                tangent=local_tangent,
                score=score,
                geometric_score=geometry,
                confidence=confidence,
                view_count=len(track.observations),
                edge_margin=edge_margin,
                local_leaf_width=local_leaf_width,
                edge_margin_ratio=edge_margin_ratio,
                clearance=clearance,
                longitudinal_ratio=longitudinal_ratio,
            ))
        results.sort(key=lambda item: item.score, reverse=True)
        return results[:max(3 * self.per_leaf_candidate_count, 12)]

    def _approach_clearance(
        self,
        point: np.ndarray,
        normal: np.ndarray,
        tangent: np.ndarray,
        other_tree: Optional[cKDTree],
    ) -> float:
        if other_tree is None:
            return self.approach_distance
        bitangent = normalized(np.cross(normal, tangent))
        direction_clearances = []
        for direction in (tangent, -tangent, bitangent, -bitangent):
            minimum = self.approach_distance
            for distance in np.linspace(
                0.015, self.approach_distance, 8
            ):
                sample = point - direction * distance
                minimum = min(
                    minimum, float(other_tree.query(sample)[0]))
            direction_clearances.append(minimum)
        # The later ROMU4O-style orientation search only needs one clear
        # in-plane approach direction; exact collision checking remains the
        # hard gate in MoveIt.
        return max(direction_clearances)

    def _publish_canopy_bounds(
        self, points: np.ndarray, cloud: PointCloud2
    ) -> None:
        if (
            self.conservative_lower is not None
            and self.conservative_upper is not None
        ):
            lower = self.conservative_lower
            upper = self.conservative_upper
        else:
            lower = np.percentile(points, 2.0, axis=0)
            upper = np.percentile(points, 98.0, axis=0)
        centre = 0.5 * (lower + upper)
        size = np.maximum(
            upper - lower, self.metric_floor)
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = cloud.header.stamp
        marker.ns = 'canopy_bounds'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = Point(
            x=float(centre[0]), y=float(centre[1]), z=float(centre[2]))
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(
            x=float(size[0]), y=float(size[1]), z=float(size[2]))
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.20
        self.bounds_publisher.publish(marker)

    def _update_conservative_bounds(self, points: np.ndarray) -> None:
        """Expand, but never shrink, the possible whole-plant envelope."""
        observed_lower = np.percentile(points, 2.0, axis=0)
        observed_upper = np.percentile(points, 98.0, axis=0)
        observed_size = np.maximum(
            observed_upper - observed_lower, self.metric_floor)
        padding = 0.05 * observed_size
        observed_lower -= padding
        observed_upper += padding
        if (
            self.conservative_lower is None
            or self.conservative_upper is None
        ):
            centre = 0.5 * (observed_lower + observed_upper)
            size = observed_upper - observed_lower
            size[:2] = np.maximum(
                (1.0 + self.canopy_expansion_margin) * size[:2],
                self.overview_minimum_span,
            )
            self.conservative_lower = centre - 0.5 * size
            self.conservative_upper = centre + 0.5 * size
        else:
            self.conservative_lower = np.minimum(
                self.conservative_lower, observed_lower)
            self.conservative_upper = np.maximum(
                self.conservative_upper, observed_upper)

        centre = 0.5 * (
            self.conservative_lower + self.conservative_upper)
        span = float(max(
            (self.conservative_upper - self.conservative_lower)[:2]))
        for _, _, target in self._unresolved_frontier_targets(
                centre, span):
            self.conservative_lower = np.minimum(
                self.conservative_lower, target - self.voxel_size)
            self.conservative_upper = np.maximum(
                self.conservative_upper, target + self.voxel_size)
        self.coarse_canopy_span = max(
            self.coarse_canopy_span,
            float(max(
                (self.conservative_upper
                 - self.conservative_lower)[:2])),
        )

    def _publish_overview_views(
        self, points: np.ndarray, cloud: PointCloud2, transform
    ) -> Tuple[int, float, float, float]:
        """Rank downward overview views by coverage, angle and motion cost."""
        if (
            self.conservative_lower is not None
            and self.conservative_upper is not None
        ):
            lower = self.conservative_lower
            upper = self.conservative_upper
        else:
            lower = np.percentile(points, 2.0, axis=0)
            upper = np.percentile(points, 98.0, axis=0)
        centre = 0.5 * (lower + upper)
        box_samples = self._box_surface_samples(lower, upper)
        envelope_radius = 0.5 * float(np.linalg.norm(upper - lower))
        aspect = float(cloud.height) / max(float(cloud.width), 1.0)
        vertical_fov = 2.0 * math.atan(
            math.tan(0.5 * self.camera_horizontal_fov) * aspect)
        limiting_fov = min(self.camera_horizontal_fov, vertical_fov)
        full_coverage_distance = (
            envelope_radius
            / max(
                math.tan(0.5 * limiting_fov)
                * self.overview_image_margin,
                1e-3,
            )
        )

        camera_position = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ])
        camera_quaternion = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ])
        camera_rotation = Rotation.from_quat(camera_quaternion)
        safe_offset = camera_position - centre
        planar_direction = safe_offset[:2]
        if np.linalg.norm(planar_direction) < self.metric_floor:
            optical_axis = camera_rotation.apply(
                np.array([0.0, 0.0, 1.0]))
            planar_direction = -optical_axis[:2]
        if np.linalg.norm(planar_direction) < self.metric_floor:
            planar_direction = np.array([-1.0, 0.0])
        planar_direction = normalized(planar_direction)

        centring_shift = centre - (
            camera_position
            + float(np.dot(
                centre - camera_position,
                camera_rotation.apply(np.array([0.0, 0.0, 1.0]))))
            * camera_rotation.apply(np.array([0.0, 0.0, 1.0]))
        )

        preferred = float(np.clip(
            self.overview_elevation_degrees,
            self.minimum_downward_pitch_degrees,
            self.maximum_downward_pitch_degrees,
        ))
        raw_elevations = (
            preferred,
            self.maximum_downward_pitch_degrees,
            0.5 * (
                preferred + self.maximum_downward_pitch_degrees),
            preferred + 10.0,
            preferred - 10.0,
            self.minimum_downward_pitch_degrees,
        )
        elevations = []
        for elevation_degrees in raw_elevations:
            elevation_degrees = float(np.clip(
                elevation_degrees,
                self.minimum_downward_pitch_degrees,
                self.maximum_downward_pitch_degrees,
            ))
            if not any(
                abs(elevation_degrees - value) < 1e-6
                for value in elevations
            ):
                elevations.append(elevation_degrees)

        ranked_views = []
        maximum_retreat = 0.0
        for elevation_degrees in elevations:
            elevation = math.radians(elevation_degrees)
            required_height = (
                upper[2] + self.minimum_camera_above_canopy - centre[2])
            for distance_scale in (1.0, 0.82, 0.68, 0.56, 0.44):
                distance = max(
                    distance_scale * full_coverage_distance,
                    required_height / max(math.sin(elevation), 1e-3),
                )
                direction = np.array([
                    math.cos(elevation) * planar_direction[0],
                    math.cos(elevation) * planar_direction[1],
                    math.sin(elevation),
                ])
                position = centre + distance * direction
                quaternion = self._look_at_quaternion(position, centre)
                if not self._valid_downward_view(
                    position, quaternion, upper[2]
                ):
                    continue
                if any(
                    np.linalg.norm(position - item[3]) < 1e-4
                    for item in ranked_views
                ):
                    continue
                coverage = self._projected_coverage(
                    position, quaternion, box_samples, cloud)
                angle_quality = 1.0 - (
                    abs(elevation_degrees - preferred)
                    / max(
                        self.maximum_downward_pitch_degrees
                        - self.minimum_downward_pitch_degrees,
                        1e-3,
                    )
                )
                motion_distance = float(np.linalg.norm(
                    position - camera_position))
                motion_cost = motion_distance / max(
                    full_coverage_distance, self.voxel_size)
                score = (
                    6.0 * coverage
                    + 1.0 * angle_quality
                    - 0.20 * motion_cost
                )
                maximum_retreat = max(maximum_retreat, motion_distance)
                ranked_views.append((
                    score,
                    coverage,
                    elevation_degrees,
                    position,
                    quaternion,
                ))

        ranked_views.sort(key=lambda item: item[0], reverse=True)
        for rank, item in enumerate(ranked_views, start=1):
            score, coverage, elevation_degrees, position, _ = item
            self.get_logger().info(
                f'Overview rank {rank}: score={score:.3f}; '
                f'coverage={coverage:.3f}; '
                f'downward_pitch={elevation_degrees:.1f} deg; '
                f'position=({position[0]:.3f}, {position[1]:.3f}, '
                f'{position[2]:.3f})')
        message = PoseArray()
        message.header.frame_id = self.target_frame
        message.header.stamp = cloud.header.stamp
        for _, _, _, position, quaternion in ranked_views:
            message.poses.append(Pose(
                position=Point(
                    x=float(position[0]),
                    y=float(position[1]),
                    z=float(position[2]),
                ),
                orientation=quaternion_message(quaternion),
            ))
        self.views_publisher.publish(message)
        return (
            len(message.poses),
            maximum_retreat,
            float(np.linalg.norm(centring_shift)),
            float(ranked_views[0][1]) if ranked_views else 0.0,
        )

    def _unresolved_frontier_targets(
        self, canopy_centre: np.ndarray, canopy_span: float
    ) -> List[Tuple[float, np.ndarray, np.ndarray]]:
        """Predict missing leaf continuation from the latest border contact."""
        targets = []
        for track in self.tracks:
            if not track.observations:
                continue
            latest = max(
                track.observations, key=lambda item: item.view_id)
            if (
                not latest.touches_border
                or latest.frontier_point is None
                or latest.frontier_direction is None
            ):
                continue
            points = self._voxel_fuse(track.observations)
            direction_3d = normalized(latest.frontier_direction)
            projection = points @ direction_3d
            visible_length = max(
                float(np.ptp(projection)), 3.0 * self.voxel_size)
            extension = min(
                0.35 * visible_length, 0.25 * canopy_span)
            target = (
                latest.frontier_point + extension * direction_3d)
            planar_direction = target[:2] - canopy_centre[:2]
            if np.linalg.norm(planar_direction) < self.voxel_size:
                planar_direction = direction_3d[:2]
            if np.linalg.norm(planar_direction) < self.voxel_size:
                continue
            planar_direction = normalized(planar_direction)
            age = max(0, self.view_count - latest.view_id)
            priority = (
                10.0
                + visible_length / max(canopy_span, self.voxel_size)
                + 0.25 * age
            )
            targets.append((priority, planar_direction, target))
        targets.sort(key=lambda item: item[0], reverse=True)
        return targets

    @staticmethod
    def _box_surface_samples(
        lower: np.ndarray, upper: np.ndarray
    ) -> np.ndarray:
        axes = [
            np.linspace(float(lower[index]), float(upper[index]), 3)
            for index in range(3)
        ]
        samples = []
        for x_index, x in enumerate(axes[0]):
            for y_index, y in enumerate(axes[1]):
                for z_index, z in enumerate(axes[2]):
                    if 1 in (x_index, y_index, z_index) and (
                        x_index == y_index == z_index == 1
                    ):
                        continue
                    samples.append((x, y, z))
        return np.asarray(samples)

    def _camera_fovs(self, cloud: PointCloud2) -> Tuple[float, float]:
        aspect = float(cloud.height) / max(float(cloud.width), 1.0)
        vertical = 2.0 * math.atan(
            math.tan(0.5 * self.camera_horizontal_fov) * aspect)
        return self.camera_horizontal_fov, vertical

    def _projected_coverage(
        self,
        camera_position: np.ndarray,
        camera_quaternion: np.ndarray,
        samples: np.ndarray,
        cloud: PointCloud2,
    ) -> float:
        inside = self._projected_inside_mask(
            camera_position, camera_quaternion, samples, cloud)
        return float(np.count_nonzero(inside) / max(samples.shape[0], 1))

    def _projected_inside_mask(
        self,
        camera_position: np.ndarray,
        camera_quaternion: np.ndarray,
        samples: np.ndarray,
        cloud: PointCloud2,
    ) -> np.ndarray:
        """Return which samples lie inside the usable camera image."""
        if not samples.size:
            return np.zeros(0, dtype=bool)
        camera_points = Rotation.from_quat(
            camera_quaternion).inv().apply(
                samples - camera_position)
        horizontal_fov, vertical_fov = self._camera_fovs(cloud)
        forward = camera_points[:, 2]
        safe_forward = np.maximum(forward, 1e-6)
        horizontal = np.abs(camera_points[:, 0]) / (
            safe_forward * math.tan(0.5 * horizontal_fov))
        vertical = np.abs(camera_points[:, 1]) / (
            safe_forward * math.tan(0.5 * vertical_fov))
        usable = max(0.20, 1.0 - self.view_frame_margin)
        inside = (
            (forward > 0.05)
            & (horizontal <= usable)
            & (vertical <= usable)
        )
        return inside

    def _completion_surface_evidence(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Build surface-quality deficits and their observed directions."""
        sample_groups = []
        weight_groups = []
        direction_groups = []
        target_support = max(
            self.required_views, self.minimum_surface_views + 1)
        support_radius = (
            self.surface_support_voxels * self.voxel_size)
        for track in self.tracks:
            if not track.observations:
                continue
            full_fused = self._voxel_fuse(track.observations)
            stride = max(1, full_fused.shape[0] // 300)
            fused = full_fused[::stride]
            support_rows = []
            direction_sum = np.zeros_like(fused)
            first_direction = np.zeros_like(fused)
            for observation in track.observations:
                distances, _ = cKDTree(observation.points).query(fused)
                supported = distances <= support_radius
                support_rows.append(supported)
                vectors = observation.camera_position - fused
                norms = np.linalg.norm(vectors, axis=1)
                valid = supported & (norms > 1e-6)
                directions = np.zeros_like(vectors)
                directions[valid] = (
                    vectors[valid] / norms[valid, None])
                direction_sum += directions
                unset = (
                    valid
                    & (np.linalg.norm(first_direction, axis=1) < 1e-6)
                )
                first_direction[unset] = directions[unset]

            support_count = np.count_nonzero(
                np.stack(support_rows, axis=0), axis=0)
            keep = (
                (support_count > 0)
                & (support_count < target_support)
            )
            if not np.any(keep):
                continue
            representative = direction_sum[keep]
            representative_norm = np.linalg.norm(
                representative, axis=1)
            fallback = first_direction[keep]
            usable = representative_norm > 1e-6
            representative[usable] /= representative_norm[usable, None]
            representative[~usable] = fallback[~usable]
            kept_points = fused[keep]

            # NBV must improve the reconstructed surface, not merely reproduce
            # its image coverage.  High local scatter, sparse neighborhoods,
            # and grazing observations all indicate geometry whose depth and
            # normal estimate still need a second view.
            quality_deficits = []
            quality_tree = cKDTree(full_fused)
            quality_radius = max(
                3.0 * self.voxel_size, self.quality_radius_floor)
            for point, observed_direction in zip(
                    kept_points, representative):
                neighbor_indices = quality_tree.query_ball_point(
                    point, quality_radius)
                if len(neighbor_indices) < 8:
                    quality_deficits.append(1.0)
                    continue
                neighborhood = full_fused[neighbor_indices]
                covariance = np.cov(neighborhood, rowvar=False)
                eigenvalues, eigenvectors = np.linalg.eigh(covariance)
                local_noise = math.sqrt(max(float(eigenvalues[0]), 0.0))
                noise_deficit = clamp01(local_noise / max(
                    self.surface_noise_reference, 1e-6))
                local_normal = eigenvectors[:, 0]
                incidence_quality = abs(float(np.dot(
                    local_normal, observed_direction)))
                grazing_deficit = clamp01(1.0 - incidence_quality)
                quality_deficits.append(
                    0.60 * noise_deficit + 0.40 * grazing_deficit)

            support_deficit = (
                (target_support - support_count[keep])
                / target_support)
            quality_deficits = np.asarray(quality_deficits)
            sample_groups.append(fused[keep])
            weight_groups.append(
                support_deficit * (0.50 + 0.50 * quality_deficits))
            direction_groups.append(representative)

        if not sample_groups:
            empty_points = np.empty((0, 3), dtype=float)
            return empty_points, np.empty(0, dtype=float), empty_points
        return (
            np.concatenate(sample_groups, axis=0),
            np.concatenate(weight_groups, axis=0),
            np.concatenate(direction_groups, axis=0),
        )

    def _expected_completion_gain(
        self,
        camera_position: np.ndarray,
        camera_quaternion: np.ndarray,
        samples: np.ndarray,
        weights: np.ndarray,
        observed_directions: np.ndarray,
        cloud: PointCloud2,
    ) -> float:
        """Estimate quality-deficient surface gained from a new direction."""
        if not samples.size:
            return 0.0
        inside = self._projected_inside_mask(
            camera_position, camera_quaternion, samples, cloud)
        candidate_vectors = camera_position - samples
        candidate_norms = np.linalg.norm(candidate_vectors, axis=1)
        valid = candidate_norms > 1e-6
        candidate_directions = np.zeros_like(candidate_vectors)
        candidate_directions[valid] = (
            candidate_vectors[valid] / candidate_norms[valid, None])
        cosine = np.sum(
            candidate_directions * observed_directions, axis=1)
        angle = np.arccos(np.clip(cosine, -1.0, 1.0))
        directional_novelty = np.clip(
            angle / math.radians(45.0), 0.0, 1.0)
        useful = weights * inside * directional_novelty
        return float(np.sum(useful) / max(np.sum(weights), 1e-9))

    def _frontier_focus(
        self, centre: np.ndarray, canopy_span: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        frontiers = self._unresolved_frontier_targets(
            centre, canopy_span)
        if not frontiers:
            return centre.copy(), np.empty((0, 3))
        weights = np.asarray([item[0] for item in frontiers])
        targets = np.asarray([item[2] for item in frontiers])
        focus = np.average(targets, axis=0, weights=weights)
        shift = focus - centre
        maximum_shift = 0.15 * canopy_span
        shift_norm = float(np.linalg.norm(shift))
        if shift_norm > maximum_shift:
            shift *= maximum_shift / shift_norm
        return centre + shift, targets

    def _publish_observation_views(
        self, points: np.ndarray, cloud: PointCloud2
    ) -> float:
        """Publish completion-driven NBV candidates after the overview."""
        if (
            self.conservative_lower is not None
            and self.conservative_upper is not None
        ):
            lower = self.conservative_lower
            upper = self.conservative_upper
        else:
            lower = np.percentile(points, 2.0, axis=0)
            upper = np.percentile(points, 98.0, axis=0)
        centre = 0.5 * (lower + upper)
        size = np.maximum(
            upper - lower, 0.02 * self.scene_scale)
        canopy_span = float(max(size[:2]))
        box_samples = self._box_surface_samples(lower, upper)
        focus, frontier_samples = self._frontier_focus(
            centre, canopy_span)

        horizontal_fov, vertical_fov = self._camera_fovs(cloud)
        usable = max(0.20, 1.0 - self.view_frame_margin)
        envelope_radius = 0.5 * float(np.linalg.norm(size))
        limiting_fov = min(horizontal_fov, vertical_fov)
        coverage_distance = max(
            envelope_radius / max(
                usable * math.tan(0.5 * limiting_fov), 1e-3),
            0.30 * self.scene_scale,
        )

        all_observations = [
            observation
            for track in self.tracks
            for observation in track.observations
        ]
        observed_directions = []
        for observation in all_observations:
            offset = observation.camera_position - centre
            if np.linalg.norm(offset) > self.metric_floor:
                observed_directions.append(normalized(offset))
        latest_observation = (
            max(all_observations, key=lambda item: item.view_id)
            if all_observations else None)
        completion_samples, completion_weights, completion_directions = (
            self._completion_surface_evidence())
        historical_positions = np.asarray([
            observation.camera_position
            for observation in all_observations
        ])

        def evaluate_view(position, quaternion, direction):
            coverage = self._projected_coverage(
                position, quaternion, box_samples, cloud)
            if coverage < 0.55:
                return None
            if frontier_samples.size:
                frontier_coverage = self._projected_coverage(
                    position, quaternion, frontier_samples, cloud)
            else:
                frontier_coverage = 0.0
            if observed_directions:
                angular_separation = min(
                    math.acos(float(np.clip(np.dot(
                        direction, observed), -1.0, 1.0)))
                    for observed in observed_directions
                )
            else:
                angular_separation = math.pi
            angular_novelty = clamp01(
                angular_separation / math.radians(45.0))
            completion_gain = self._expected_completion_gain(
                position,
                quaternion,
                completion_samples,
                completion_weights,
                completion_directions,
                cloud,
            )
            frontier_gain = frontier_coverage * (
                0.25 + 0.75 * angular_novelty)
            if latest_observation is not None:
                movement = float(np.linalg.norm(
                    position - latest_observation.camera_position))
                motion_cost = movement / max(
                    coverage_distance, self.voxel_size)
                reachability_prior = math.exp(-movement / max(
                    self.reachability_decay_length, 1e-6))
                history_distance = float(np.min(np.linalg.norm(
                    historical_positions - position, axis=1)))
            else:
                movement = 0.0
                motion_cost = 0.0
                reachability_prior = 0.0
                history_distance = float('inf')

            redundant = (
                angular_separation
                < self.minimum_nbv_angular_separation
                and history_distance < self.minimum_nbv_translation
            )
            if (
                redundant
                and completion_gain < self.minimum_nbv_completion_gain
                and frontier_gain < 0.25
            ):
                return None
            redundancy_penalty = 1.0 if redundant else 0.0
            coverage_guard = (
                1.0
                if coverage >= self.minimum_view_coverage
                else coverage
                / max(self.minimum_view_coverage, 1e-3)
            )
            score = (
                5.0 * completion_gain
                + 2.0 * frontier_gain
                + 1.2 * angular_novelty
                + 1.0 * coverage_guard
                + 0.8 * reachability_prior
                - 0.25 * motion_cost
                - 2.0 * redundancy_penalty
            )
            return (
                score,
                completion_gain,
                coverage,
                frontier_gain,
                angular_separation,
                movement,
                position,
                quaternion,
            )

        distance_tiers = (
            (1.00, 8),
            (0.82, 4),
            (0.68, 4),
        )
        ranked_tiers = []
        for distance_scale, tier_limit in distance_tiers:
            tier = []
            for elevation_degrees in (45.0, 60.0, 75.0, 90.0):
                elevation = math.radians(elevation_degrees)
                required_height = (
                    upper[2] + self.minimum_camera_above_canopy - centre[2])
                shell_distance = max(
                    distance_scale * coverage_distance,
                    required_height / max(math.sin(elevation), 1e-3),
                    0.24 * self.scene_scale,
                )
                azimuth_count = (
                    1 if abs(elevation_degrees - 90.0) < 1e-6 else 8)
                for azimuth_index in range(azimuth_count):
                    azimuth = 2.0 * math.pi * azimuth_index / 8.0
                    direction = np.array([
                        math.cos(elevation) * math.cos(azimuth),
                        math.cos(elevation) * math.sin(azimuth),
                        math.sin(elevation),
                    ])
                    position = centre + shell_distance * direction
                    best = None
                    for target in (centre, focus):
                        quaternion = self._look_at_quaternion(
                            position, target)
                        if not self._valid_downward_view(
                            position, quaternion, upper[2]
                        ):
                            continue
                        candidate = evaluate_view(
                            position, quaternion, direction)
                        if candidate is None:
                            continue
                        if best is None or candidate[0] > best[0]:
                            best = candidate
                    if best is not None:
                        tier.append(best)
            tier.sort(key=lambda item: item[0], reverse=True)
            ranked_tiers.extend(tier[:tier_limit])

        # A globally attractive shell can still lie just outside the arm's
        # workspace. Seed small azimuth changes around the latest pose, which
        # is known to be reachable, while retaining the same downward-view
        # and canopy-clearance constraints.
        local_views = []
        if latest_observation is not None:
            current_offset = latest_observation.camera_position - centre
            current_radius = float(np.linalg.norm(current_offset))
            planar_radius = float(np.linalg.norm(current_offset[:2]))
            if (
                current_radius > self.local_view_radius_gate
                and planar_radius > self.local_view_planar_gate
            ):
                current_azimuth = math.atan2(
                    current_offset[1], current_offset[0])
                current_elevation = math.asin(float(np.clip(
                    current_offset[2] / current_radius, -1.0, 1.0)))
                current_elevation_degrees = math.degrees(current_elevation)
                seed_elevation_degrees = float(np.clip(
                    current_elevation_degrees,
                    self.minimum_downward_pitch_degrees,
                    self.maximum_downward_pitch_degrees,
                ))
                required_height = (
                    upper[2] + self.minimum_camera_above_canopy
                    - centre[2]
                )
                seed_elevation = math.radians(seed_elevation_degrees)
                base_local_radius = max(
                    current_radius,
                    required_height / max(math.sin(seed_elevation), 1e-3),
                )
                for elevation_delta_degrees in (0.0, -5.0, 5.0, -10.0, 10.0):
                    elevation_degrees = float(np.clip(
                        seed_elevation_degrees + elevation_delta_degrees,
                        self.minimum_downward_pitch_degrees,
                        self.maximum_downward_pitch_degrees,
                    ))
                    elevation = math.radians(elevation_degrees)
                    minimum_radius = required_height / max(
                        math.sin(elevation), 1e-3)
                    for radius_scale in (0.85, 1.0, 1.15):
                        local_radius = max(
                            radius_scale * base_local_radius,
                            minimum_radius,
                            0.24 * self.scene_scale,
                        )
                        for azimuth_delta_degrees in (
                            0.0, -5.0, 5.0, -10.0, 10.0, -15.0, 15.0
                        ):
                            azimuth = current_azimuth + math.radians(
                                azimuth_delta_degrees)
                            direction = np.array([
                                math.cos(elevation) * math.cos(azimuth),
                                math.cos(elevation) * math.sin(azimuth),
                                math.sin(elevation),
                            ])
                            position = centre + local_radius * direction
                            for target in (centre, focus):
                                quaternion = self._look_at_quaternion(
                                    position, target)
                                if not self._valid_downward_view(
                                    position, quaternion, upper[2]
                                ):
                                    continue
                                candidate = evaluate_view(
                                    position, quaternion, direction)
                                if candidate is not None:
                                    local_views.append(candidate)

                radial_direction = normalized(current_offset)
                lateral_direction = normalized(np.array([
                    -current_offset[1], current_offset[0], 0.0,
                ]))
                if np.linalg.norm(lateral_direction) < 1e-6:
                    lateral_direction = np.array([0.0, 1.0, 0.0])
                vertical_direction = np.array([0.0, 0.0, 1.0])
                micro_step = 0.04 * self.scene_scale
                retreat_step = 0.06 * self.scene_scale
                direct_offsets = (
                    np.zeros(3),
                    micro_step * lateral_direction,
                    -micro_step * lateral_direction,
                    0.75 * micro_step * vertical_direction,
                    -0.75 * micro_step * vertical_direction,
                    -retreat_step * radial_direction,
                    -retreat_step * radial_direction
                    + 0.5 * micro_step * lateral_direction,
                    -retreat_step * radial_direction
                    - 0.5 * micro_step * lateral_direction,
                )
                for offset in direct_offsets:
                    position = latest_observation.camera_position + offset
                    for target in (centre, focus):
                        quaternion = self._look_at_quaternion(
                            position, target)
                        if not self._valid_downward_view(
                            position, quaternion, upper[2]
                        ):
                            continue
                        direction = normalized(position - centre)
                        candidate = evaluate_view(
                            position, quaternion, direction)
                        if candidate is not None:
                            local_views.append(candidate)

        ranked_tiers.extend(local_views)
        ranked_tiers.sort(key=lambda item: item[0], reverse=True)
        unique_views = []
        for candidate in ranked_tiers:
            if any(
                np.linalg.norm(candidate[6] - accepted[6]) < 1e-4
                for accepted in unique_views
            ):
                continue
            unique_views.append(candidate)
        ranked_tiers = unique_views[:24]
        self.get_logger().info(
            f'NBV candidates={len(ranked_tiers)}; '
            f'local_reachable_seeds={len(local_views)}; '
            f'best_quality_completion_gain='
            f'{ranked_tiers[0][1] if ranked_tiers else 0.0:.3f}; '
            f'best_frame_coverage='
            f'{ranked_tiers[0][2] if ranked_tiers else 0.0:.3f}')
        for rank, candidate in enumerate(ranked_tiers[:5], start=1):
            self.get_logger().info(
                f'NBV rank {rank}: score={candidate[0]:.3f}; '
                f'quality_completion_gain={candidate[1]:.3f}; '
                f'coverage={candidate[2]:.3f}; '
                f'frontier_gain={candidate[3]:.3f}; '
                f'angle={math.degrees(candidate[4]):.1f} deg; '
                f'movement={candidate[5]:.3f} m; '
                f'position=({candidate[6][0]:.3f}, '
                f'{candidate[6][1]:.3f}, {candidate[6][2]:.3f})')

        message = PoseArray()
        message.header.frame_id = self.target_frame
        message.header.stamp = cloud.header.stamp
        for candidate in ranked_tiers:
            position = candidate[6]
            quaternion = candidate[7]
            message.poses.append(Pose(
                position=Point(
                    x=float(position[0]),
                    y=float(position[1]),
                    z=float(position[2]),
                ),
                orientation=quaternion_message(quaternion),
            ))
        self.views_publisher.publish(message)
        if not ranked_tiers:
            return 0.0
        return float(ranked_tiers[0][1])

    @staticmethod
    def _look_at_quaternion(
        camera_position: np.ndarray, target: np.ndarray
    ) -> np.ndarray:
        optical_z = normalized(target - camera_position)
        world_up = np.array([0.0, 0.0, 1.0])
        optical_x = np.cross(optical_z, world_up)
        if np.linalg.norm(optical_x) < 1e-6:
            optical_x = np.array([1.0, 0.0, 0.0])
        optical_x = normalized(optical_x)
        optical_y = normalized(np.cross(optical_z, optical_x))
        matrix = np.column_stack((optical_x, optical_y, optical_z))
        return Rotation.from_matrix(matrix).as_quat()

    def _valid_downward_view(
        self,
        camera_position: np.ndarray,
        camera_quaternion: np.ndarray,
        canopy_top: float,
    ) -> bool:
        if (
            camera_position[2]
            < canopy_top + self.minimum_camera_above_canopy - 1e-6
        ):
            return False
        optical_z = Rotation.from_quat(camera_quaternion).apply(
            np.array([0.0, 0.0, 1.0]))
        downward_pitch = math.degrees(math.asin(float(np.clip(
            -optical_z[2], -1.0, 1.0))))
        return (
            self.minimum_downward_pitch_degrees - 1e-6
            <= downward_pitch
            <= self.maximum_downward_pitch_degrees + 1e-6
        )

    def _candidate_orientation_base(
        self, candidate: GraspCandidate
    ) -> Rotation:
        # The RG2 closes along its local X axis.  Keep that axis aligned with
        # the leaf normal so the two fingertips straddle the leaf thickness,
        # while the local Z approach axis stays in the leaf plane.  This
        # matches the calibrated hand-authored leaf pinch pose.
        gripper_x = normalized(candidate.normal)
        gripper_y = candidate.tangent - (
            np.dot(candidate.tangent, gripper_x) * gripper_x)
        gripper_y = normalized(gripper_y)
        gripper_z = normalized(np.cross(gripper_x, gripper_y))
        gripper_y = normalized(np.cross(gripper_z, gripper_x))
        return Rotation.from_matrix(
            np.column_stack((gripper_x, gripper_y, gripper_z)))

    def _candidate_orientations(
        self, candidate: GraspCandidate
    ) -> List[np.ndarray]:
        base = self._candidate_orientation_base(candidate)
        return [
            (base * Rotation.from_euler('X', angle, degrees=True)).as_quat()
            for angle in (0.0, -45.0, -90.0, -135.0, -180.0)
        ]

    def _reachability_orientations(
        self, candidate: GraspCandidate
    ) -> List[Tuple[np.ndarray, float, float]]:
        base = self._candidate_orientation_base(candidate)
        orientations = []
        for tilt in (
            0.0, -10.0, 10.0, -20.0, 20.0, -30.0, 30.0
        ):
            tilted = base * Rotation.from_euler('Y', tilt, degrees=True)
            # Test the four orthogonal in-plane approach directions before
            # diagonals.  +90 deg is not interchangeable with -90 deg here:
            # it reverses the Cartesian approach direction along the leaf.
            for roll in (
                0.0, -90.0, -180.0, 90.0, -45.0, -135.0
            ):
                quaternion = (
                    tilted * Rotation.from_euler(
                        'X', roll, degrees=True)).as_quat()
                orientations.append((quaternion, tilt, roll))
        return orientations

    def _publish_candidates(
        self, candidates: Sequence[GraspCandidate], cloud: PointCloud2
    ) -> None:
        message = LeafPoseArrays()
        message.header.frame_id = self.target_frame
        message.header.stamp = cloud.header.stamp
        pose_groups = [[], [], [], [], []]
        for point_index, candidate in enumerate(candidates):
            orientations = self._candidate_orientations(candidate)
            for group, quaternion in zip(pose_groups, orientations):
                group.append(Pose(
                    position=Point(
                        x=float(candidate.point[0]),
                        y=float(candidate.point[1]),
                        z=float(candidate.point[2]),
                    ),
                    orientation=quaternion_message(quaternion),
                ))
            message.leaf_ids.append(candidate.leaf_id)
            message.scores.append(float(candidate.score))
            message.geometric_scores.append(
                float(candidate.geometric_score))
            message.confidences.append(float(candidate.confidence))
            message.view_counts.append(candidate.view_count)
            message.edge_margins.append(float(candidate.edge_margin))
            message.local_leaf_widths.append(
                float(candidate.local_leaf_width))
            message.edge_margin_ratios.append(
                float(candidate.edge_margin_ratio))
            message.approach_clearances.append(float(candidate.clearance))
            message.longitudinal_ratios.append(
                float(candidate.longitudinal_ratio))
            message.projection_distances.append(
                float(candidate.projection_distance))
            message.collision_leaf_ids.append(
                candidate.collision_leaf_id)
            for quaternion, tilt, roll in self._reachability_orientations(
                candidate
            ):
                message.candidate_poses.append(Pose(
                    position=Point(
                        x=float(candidate.point[0]),
                        y=float(candidate.point[1]),
                        z=float(candidate.point[2]),
                    ),
                    orientation=quaternion_message(quaternion),
                ))
                message.candidate_point_indices.append(point_index)
                message.candidate_tilt_degrees.append(float(tilt))
                message.candidate_roll_degrees.append(float(roll))
        (
            message.poses1,
            message.poses2,
            message.poses3,
            message.poses4,
            message.poses5,
        ) = pose_groups
        message.selection_method = (
            'distal_same_leaf_projection_then_romu4o_reachability')
        self.candidates_publisher.publish(message)
        self._publish_candidate_markers(candidates, cloud)
        self.get_logger().info(
            f'Published {len(candidates)} ranked multi-view grasp points')

    def _publish_candidate_markers(
        self, candidates: Sequence[GraspCandidate], cloud: PointCloud2
    ) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.target_frame
        clear.header.stamp = cloud.header.stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, candidate in enumerate(candidates):
            sphere = Marker()
            sphere.header.frame_id = self.target_frame
            sphere.header.stamp = cloud.header.stamp
            sphere.ns = 'ranked_grasp_points'
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(
                x=float(candidate.point[0]),
                y=float(candidate.point[1]),
                z=float(candidate.point[2]),
            )
            sphere.pose.orientation.w = 1.0
            marker_scale = 0.014 * self.scene_scale
            sphere.scale = Vector3(
                x=marker_scale, y=marker_scale, z=marker_scale)
            sphere.color.r = 0.05
            sphere.color.g = 0.85
            sphere.color.b = 0.25
            sphere.color.a = 1.0
            markers.markers.append(sphere)
            label = Marker()
            label.header = sphere.header
            label.ns = 'projected_grasp_labels'
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(
                x=float(candidate.point[0]),
                y=float(candidate.point[1]),
                z=float(candidate.point[2] + 0.025 * self.scene_scale),
            )
            label.pose.orientation.w = 1.0
            label.scale.z = 0.025 * self.scene_scale
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(index + 1)
            markers.markers.append(label)
        self.markers_publisher.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiViewLeafPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
