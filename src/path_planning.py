from __future__ import annotations

from typing import List, Optional
import math

from .models import CarPose, Cone, Path2D


class PathPlanning:

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        
        blue_cones = [c for c in self.cones if c.color == 1]
        yellow_cones = [c for c in self.cones if c.color == 0] 
        
        step = 0.4
        num_points = 25
        
        blue_forward = self._filter_and_sort_cones(blue_cones)
        yellow_forward = self._filter_and_sort_cones(yellow_cones)
        
        if not blue_forward and not yellow_forward:
            return self._path_straight(step, num_points)
        
        waypoints = self._compute_waypoints(blue_forward, yellow_forward)
        
        if not waypoints:
            return self._path_straight(step, num_points)
        
        full_waypoints = [(self.car_pose.x, self.car_pose.y)] + waypoints
        
        path = self._interpolate_path(full_waypoints, step, num_points)
        
        return path
    
    def _filter_and_sort_cones(self, cones: List[Cone]) -> List[Cone]:
        """Filter cones ahead of the car and sort by distance."""
        forward = []
        for cone in cones:
            dx = cone.x - self.car_pose.x
            dy = cone.y - self.car_pose.y
            dist = math.hypot(dx, dy)
            
            if dist < 0.5: 
                continue
            
            forward_dist = dx * math.cos(self.car_pose.yaw) + dy * math.sin(self.car_pose.yaw)
            
            if forward_dist > -0.5:
                forward.append(cone)
        
        forward.sort(key=lambda c: math.hypot(c.x - self.car_pose.x, c.y - self.car_pose.y))
        return forward

    def _get_boundary_angle(self, cone1: Cone, cone2: Cone) -> float:
        """Calculates the angle from cone1 to cone2."""
        dx = cone2.x - cone1.x
        dy = cone2.y - cone1.y
        
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return self.car_pose.yaw
        
        return math.atan2(dy, dx)
    
    def _fit_track_boundary(self, cones: List[Cone]) -> tuple[float, float]:

        if len(cones) < 2:
            return self.car_pose.yaw, 0.0
        
        xs = [c.x for c in cones]
        ys = [c.y for c in cones]
        
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        
        cov_xx = sum((x - mean_x) ** 2 for x in xs) / len(xs)
        cov_yy = sum((y - mean_y) ** 2 for y in ys) / len(ys)
        cov_xy = sum((xs[i] - mean_x) * (ys[i] - mean_y) for i in range(len(xs))) / len(xs)
        
        if abs(cov_xy) < 1e-6 and abs(cov_xx - cov_yy) < 1e-6:
            angle = self.car_pose.yaw
        else:
            angle = 0.5 * math.atan2(2 * cov_xy, cov_xx - cov_yy)

            dx_avg = xs[-1] - xs[0] if len(xs) >= 2 else math.cos(self.car_pose.yaw)
            dy_avg = ys[-1] - ys[0] if len(ys) >= 2 else math.sin(self.car_pose.yaw)
            forward_angle = math.atan2(dy_avg, dx_avg)
            

            angle_diff = (forward_angle - angle + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_diff) > math.pi / 2:
                angle += math.pi
        
        return angle, 0.0
    
    def _compute_waypoints(self, blue: List[Cone], yellow: List[Cone]) -> List[tuple[float, float]]:

        waypoints = []
        HALF_LANE = 2.5
        FORWARD_STEP = 2.5
        
        if blue and yellow:
            B1 = blue[0]
            Y1 = yellow[0]
            
            W1x = (B1.x + Y1.x) / 2
            W1y = (B1.y + Y1.y) / 2
            waypoints.append((W1x, W1y))
            
            if len(blue) >= 2 and len(yellow) >= 2:
                B2 = blue[1]
                Y2 = yellow[1]
                W2x = (B2.x + Y2.x) / 2
                W2y = (B2.y + Y2.y) / 2
                waypoints.append((W2x, W2y))
                
            elif len(blue) >= 2:
                B2 = blue[1]
                
                W2_option1_x = (B2.x + Y1.x) / 2
                W2_option1_y = (B2.y + Y1.y) / 2
                
                across_x = B1.x - Y1.x
                across_y = B1.y - Y1.y
                forward_x = across_y
                forward_y = -across_x
                forward_len = math.hypot(forward_x, forward_y)
                if forward_len > 0.1:
                    forward_x /= forward_len
                    forward_y /= forward_len
                else:
                    forward_x = math.cos(self.car_pose.yaw)
                    forward_y = math.sin(self.car_pose.yaw)
                
                W2_option2_x = W1x + forward_x * FORWARD_STEP
                W2_option2_y = W1y + forward_y * FORWARD_STEP
                
                W2x = 0.7 * W2_option1_x + 0.3 * W2_option2_x
                W2y = 0.7 * W2_option1_y + 0.3 * W2_option2_y
                waypoints.append((W2x, W2y))
                
            elif len(yellow) >= 2:
                Y2 = yellow[1]
                
                W2_option1_x = (B1.x + Y2.x) / 2
                W2_option1_y = (B1.y + Y2.y) / 2

                across_x = B1.x - Y1.x
                across_y = B1.y - Y1.y
                forward_x = across_y
                forward_y = -across_x
                forward_len = math.hypot(forward_x, forward_y)
                if forward_len > 0.1:
                    forward_x /= forward_len
                    forward_y /= forward_len
                else:
                    forward_x = math.cos(self.car_pose.yaw)
                    forward_y = math.sin(self.car_pose.yaw)
                
                W2_option2_x = W1x + forward_x * FORWARD_STEP
                W2_option2_y = W1y + forward_y * FORWARD_STEP

                W2x = 0.7 * W2_option1_x + 0.3 * W2_option2_x
                W2y = 0.7 * W2_option1_y + 0.3 * W2_option2_y
                waypoints.append((W2x, W2y))
                
            else:
                W2x = W1x + math.cos(self.car_pose.yaw) * FORWARD_STEP
                W2y = W1y + math.sin(self.car_pose.yaw) * FORWARD_STEP
                waypoints.append((W2x, W2y))
        
        elif blue:
            if len(blue) >= 3:
                boundary_angle, _ = self._fit_track_boundary(blue[:3])
                offset_angle = boundary_angle - math.pi / 2  
                
                cone1 = blue[0]
                cone2 = blue[1]
                cone3 = blue[2]
                
                
                proj1 = (cone1.x - cone1.x) * math.cos(boundary_angle) + \
                        (cone1.y - cone1.y) * math.sin(boundary_angle)
                proj2 = (cone2.x - cone1.x) * math.cos(boundary_angle) + \
                        (cone2.y - cone1.y) * math.sin(boundary_angle)
                proj3 = (cone3.x - cone1.x) * math.cos(boundary_angle) + \
                        (cone3.y - cone1.y) * math.sin(boundary_angle)
                
                W1x = cone1.x + proj1 * math.cos(boundary_angle) + math.cos(offset_angle) * HALF_LANE
                W1y = cone1.y + proj1 * math.sin(boundary_angle) + math.sin(offset_angle) * HALF_LANE
                waypoints.append((W1x, W1y))
                
                W2x = cone1.x + proj2 * math.cos(boundary_angle) + math.cos(offset_angle) * HALF_LANE
                W2y = cone1.y + proj2 * math.sin(boundary_angle) + math.sin(offset_angle) * HALF_LANE
                waypoints.append((W2x, W2y))
                
                W3x = cone1.x + proj3 * math.cos(boundary_angle) + math.cos(offset_angle) * HALF_LANE
                W3y = cone1.y + proj3 * math.sin(boundary_angle) + math.sin(offset_angle) * HALF_LANE
                waypoints.append((W3x, W3y))
                
            elif len(blue) >= 2:
                cone1 = blue[0]
                cone2 = blue[1]
                boundary_angle = self._get_boundary_angle(cone1, cone2)
                offset_angle = boundary_angle - math.pi / 2
                mid_x = (cone1.x + cone2.x) / 2
                mid_y = (cone1.y + cone2.y) / 2
                W1x = mid_x + math.cos(offset_angle) * HALF_LANE
                W1y = mid_y + math.sin(offset_angle) * HALF_LANE
                waypoints.append((W1x, W1y))
                W2x = W1x + math.cos(boundary_angle) * FORWARD_STEP
                W2y = W1y + math.sin(boundary_angle) * FORWARD_STEP
                waypoints.append((W2x, W2y))
            else:
                cone = blue[0]
                path_angle = self.car_pose.yaw
                offset_angle = path_angle - math.pi / 2
                W1x = cone.x + math.cos(offset_angle) * HALF_LANE
                W1y = cone.y + math.sin(offset_angle) * HALF_LANE
                waypoints.append((W1x, W1y))
                W2x = W1x + math.cos(path_angle) * FORWARD_STEP
                W2y = W1y + math.sin(path_angle) * FORWARD_STEP
                waypoints.append((W2x, W2y))
        
        elif yellow:
            if len(yellow) >= 3:
                boundary_angle, _ = self._fit_track_boundary(yellow[:3])
                offset_angle = boundary_angle + math.pi / 2  
                
                cone1 = yellow[0]
                cone2 = yellow[1]
                cone3 = yellow[2]
                
                proj1 = (cone1.x - cone1.x) * math.cos(boundary_angle) + \
                        (cone1.y - cone1.y) * math.sin(boundary_angle)
                proj2 = (cone2.x - cone1.x) * math.cos(boundary_angle) + \
                        (cone2.y - cone1.y) * math.sin(boundary_angle)
                proj3 = (cone3.x - cone1.x) * math.cos(boundary_angle) + \
                        (cone3.y - cone1.y) * math.sin(boundary_angle)
                
                W1x = cone1.x + proj1 * math.cos(boundary_angle) + math.cos(offset_angle) * HALF_LANE
                W1y = cone1.y + proj1 * math.sin(boundary_angle) + math.sin(offset_angle) * HALF_LANE
                waypoints.append((W1x, W1y))
                
                W2x = cone1.x + proj2 * math.cos(boundary_angle) + math.cos(offset_angle) * HALF_LANE
                W2y = cone1.y + proj2 * math.sin(boundary_angle) + math.sin(offset_angle) * HALF_LANE
                waypoints.append((W2x, W2y))
                
                W3x = cone1.x + proj3 * math.cos(boundary_angle) + math.cos(offset_angle) * HALF_LANE
                W3y = cone1.y + proj3 * math.sin(boundary_angle) + math.sin(offset_angle) * HALF_LANE
                waypoints.append((W3x, W3y))
                
            elif len(yellow) >= 2:
                cone1 = yellow[0]
                cone2 = yellow[1]
                boundary_angle = self._get_boundary_angle(cone1, cone2)
                offset_angle = boundary_angle + math.pi / 2
                mid_x = (cone1.x + cone2.x) / 2
                mid_y = (cone1.y + cone2.y) / 2
                W1x = mid_x + math.cos(offset_angle) * HALF_LANE
                W1y = mid_y + math.sin(offset_angle) * HALF_LANE
                waypoints.append((W1x, W1y))
                W2x = W1x + math.cos(boundary_angle) * FORWARD_STEP
                W2y = W1y + math.sin(boundary_angle) * FORWARD_STEP
                waypoints.append((W2x, W2y))
            else:
                cone = yellow[0]
                path_angle = self.car_pose.yaw
                offset_angle = path_angle + math.pi / 2
                W1x = cone.x + math.cos(offset_angle) * HALF_LANE
                W1y = cone.y + math.sin(offset_angle) * HALF_LANE
                waypoints.append((W1x, W1y))
                W2x = W1x + math.cos(path_angle) * FORWARD_STEP
                W2y = W1y + math.sin(path_angle) * FORWARD_STEP
                waypoints.append((W2x, W2y))
        
        return waypoints
    

    def _interpolate_path(self, waypoints: List[tuple[float, float]], step: float, num_points: int) -> Path2D:
        if len(waypoints) < 2:
            return []
        
        path: Path2D = []
        
        for i in range(len(waypoints) - 1):
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i + 1]
            
            segment_len = math.hypot(x2 - x1, y2 - y1)
            if segment_len < 1e-6:
                continue
            
            num_steps = max(2, int(segment_len / step) + 1)
            
            for j in range(num_steps):
                t = j / (num_steps - 1) if num_steps > 1 else 0
                
                px = x1 + t * (x2 - x1)
                py = y1 + t * (y2 - y1)
                
                if not path or math.hypot(px - path[-1][0], py - path[-1][1]) > 1e-6:
                    path.append((px, py))
                    
                    if len(path) >= num_points:
                        return path[:num_points]
        
        if len(path) < num_points and len(path) >= 2:
            path = self._extend_path(path, step, num_points)
        
        return path[:num_points]
    
    def _extend_path(self, path: Path2D, step: float, num_points: int) -> Path2D:
        if len(path) < 2:
            return path
        
        x1, y1 = path[-2]
        x2, y2 = path[-1]
        
        if math.hypot(x2 - x1, y2 - y1) < 1e-6:
            angle = self.car_pose.yaw
        else:
            angle = math.atan2(y2 - y1, x2 - x1)
        
        while len(path) < num_points:
            last_x, last_y = path[-1]
            new_x = last_x + math.cos(angle) * step
            new_y = last_y + math.sin(angle) * step
            path.append((new_x, new_y))
        
        return path
    
    def _path_straight(self, step: float, num_points: int) -> Path2D:
        path: Path2D = []
        for i in range(1, num_points + 1):
            dx = math.cos(self.car_pose.yaw) * step * i
            dy = math.sin(self.car_pose.yaw) * step * i
            path.append((self.car_pose.x + dx, self.car_pose.y + dy))
        return path