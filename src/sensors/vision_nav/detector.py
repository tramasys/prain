import cv2
import numpy as np
from sensors.vision_nav.edge import EdgeCandidate

class Detector:
    frame_count_since_last_node = 0

    def __init__(self, frame, debug=False) -> None:
        self.__frame = frame
        self.debug = debug

    def get_edges(self):
        self.__frame = cv2.cvtColor(self.__frame, cv2.COLOR_BGR2HSV)
        self.__frame, node_detected, circle_count = self.__detect_node()
        
        if node_detected:
            Detector.frame_count_since_last_node = 0
        else:
            Detector.frame_count_since_last_node += 1

        self.__frame, edges = self.__extract_straight_sharp_edges(self.__frame, node_detected)
        if self.debug:
            self.__frame = self.__draw_node_detected_signal(self.__frame, node_detected)
        return self.__frame, edges, circle_count

    def __detect_node(self, blur_kernel=(5, 5)):
        _, _, v_channel = cv2.split(self.__frame)
        blurred = cv2.GaussianBlur(v_channel, blur_kernel, 2)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.4,
            minDist=1,
            param1=100,
            param2=40,
            minRadius=int(0.05 * self.__frame.shape[0]),
            maxRadius=int(0.35 * self.__frame.shape[0])
        )
        circle_count = len(circles) if circles is not None else 0

        output_img = cv2.cvtColor(self.__frame, cv2.COLOR_HSV2BGR)
        node_detected = False

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")  # zurückskalieren
            circles = self.__filter_circles_by_std(circles, std_factor=1)

            if len(circles) == 0:
                return output_img, False, 0

            avg_x = int(np.mean(circles[:, 0]))
            avg_y = int(np.mean(circles[:, 1]))
            avg_r = int(np.mean(circles[:, 2]))

            img_center_x = output_img.shape[0] // 2
            img_center_y = output_img.shape[1] // 2
            tolerance_x = 0.1 * output_img.shape[0]
            tolerance_y = 0.5 * output_img.shape[1]
            if abs(avg_x - img_center_x) > tolerance_x:
                return output_img, False, 0
            if abs(avg_y - img_center_y) > tolerance_y:
                return output_img, False, 0

            node_detected = True

            # if self.debug:
            #     for circle in circles:
            #         x, y, r = circle
            #         cv2.circle(output_img, (x, y), r, (0, 255, 0), 2)
 
            #     cv2.circle(output_img, (avg_x, avg_y), avg_r, (255, 255, 0), 2)
            #     cv2.circle(output_img, (avg_x, avg_y), 2, (0, 0, 255), 3)

        return output_img, node_detected, circle_count

    def __filter_circles_by_std(self, circles, std_factor=1.0):
        x, y = circles[:, 0], circles[:, 1]
        mean_x, std_x = np.mean(x), np.std(x)
        mean_y, std_y = np.mean(y), np.std(y)

        filtered = [
            (xi, yi, ri)
            for xi, yi, ri in circles
            if abs(xi - mean_x) <= std_factor * std_x and abs(yi - mean_y) <= std_factor * std_y
        ]
        return np.array(filtered)

    def __extract_straight_sharp_edges(self, image, node_detected,
                                       canny_threshold1=30, canny_threshold2=80,
                                       hough_threshold=90, min_line_length=15, max_line_gap=40):
        if not node_detected:
            return image, []

        v_channel = image[:, :, 2]
        edges = cv2.Canny(v_channel, canny_threshold1, canny_threshold2)
        output_img = image.copy()

        lines = cv2.HoughLinesP(
            edges,
            rho=1.3,
            theta=np.pi / 180,
            threshold=hough_threshold,
            minLineLength=min_line_length,
            maxLineGap=max_line_gap
        )

        if lines is None:
            return output_img, []

        lines = np.squeeze(lines)
        if len(lines.shape) == 1:
            lines = np.expand_dims(lines, axis=0)

        lines = self.__filter_lines_near_border(lines, output_img.shape)
        lines = self.__cluster_and_average_lines(lines)
        edges = [EdgeCandidate(line, center=output_img.shape[:2]) for line in lines]

        if self.debug:
            for x1, y1, x2, y2 in lines:
                cv2.line(output_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        return output_img, edges

    def __draw_node_detected_signal(self, output_img, node_detected):
        if len(output_img.shape) == 2 or output_img.shape[2] == 1:
            output_img = cv2.cvtColor(output_img, cv2.COLOR_GRAY2BGR)
        color = (0, 255, 0) if node_detected else (0, 0, 255)
        cv2.circle(output_img, (50, 50), 20, color, -1)
        return output_img

    def __filter_lines_near_border(self, lines, img_shape, tolerance=75):
        h, w = img_shape[:2]

        def dist(pt):
            x, y = pt
            return min(x, w - x, y, h - y)

        return [line for line in lines if dist(line[:2]) < tolerance or dist(line[2:]) < tolerance]

    def __cluster_and_average_lines(self, lines, angle_thresh_deg=20, dist_thresh=100, min_cluster_size=2):
        def center(line): return np.array([(line[0] + line[2]) / 2, (line[1] + line[3]) / 2])
        def direction(line):
            dx, dy = line[2] - line[0], line[3] - line[1]
            norm = np.hypot(dx, dy)
            return np.array([dx, dy]) / (norm + 1e-6)

        clusters = []

        for line in lines:
            matched = False
            dir_vec = direction(line)
            ctr = center(line)

            for cluster in clusters:
                ref_dir, ref_ctr, cluster_lines = cluster
                angle_cos = np.dot(ref_dir, dir_vec)
                angle_deg = np.degrees(np.arccos(np.clip(angle_cos, -1.0, 1.0)))
                if angle_deg < angle_thresh_deg and np.linalg.norm(ctr - ref_ctr) < dist_thresh:
                    cluster_lines.append(line)
                    cluster[0][:] = np.mean([direction(l) for l in cluster_lines], axis=0)
                    cluster[0] /= np.linalg.norm(cluster[0]) + 1e-6
                    cluster[1][:] = np.mean([center(l) for l in cluster_lines], axis=0)
                    matched = True
                    break

            if not matched:
                clusters.append([dir_vec.copy(), ctr.copy(), [line]])

        averaged = []
        for dir_vec, ctr, lines in clusters:
            if len(lines) < min_cluster_size:
                continue
            points = np.array([[l[0], l[1]] for l in lines] + [[l[2], l[3]] for l in lines])
            t_vals = np.dot(points - ctr, dir_vec)
            t_min, t_max = np.min(t_vals), np.max(t_vals)
            pt1 = (int(ctr[0] + t_min * dir_vec[0]), int(ctr[1] + t_min * dir_vec[1]))
            pt2 = (int(ctr[0] + t_max * dir_vec[0]), int(ctr[1] + t_max * dir_vec[1]))
            averaged.append((*pt1, *pt2))

        return averaged
    
    def update_frame(self, frame):
        self.__frame = frame.copy()

    @staticmethod
    def distinct_node_detected(threshold: int = 20):
        return Detector.frame_count_since_last_node >= threshold
