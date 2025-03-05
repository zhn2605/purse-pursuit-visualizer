import numpy as np

class PurePursuit:
    def calc_lookahead_pos(self, car, track):
        min_dist = float('inf')
        lookAheadPoint = None

        # Convert track points into a NumPy array
        track_points = np.column_stack((track.xs, track.ys))

        # Find the index of the closest track point to the car
        distances = np.linalg.norm(track_points - car.position, axis=1)
        closest_index = np.argmin(distances)  # Get index of closest point

        # Iterate only over future points (ensuring progression)
        for i in range(closest_index + 1, len(track.xs)):  # Start after closest point
            point = np.array([track.xs[i], track.ys[i]])
            distance = np.linalg.norm(point - car.position)

            if distance >= car.lookAheadDistance and distance < min_dist:
                min_dist = distance
                lookAheadPoint = point

        return lookAheadPoint if lookAheadPoint is not None else car.lookAheadPosition

         
    def calc_angle(self, car, track):
        lookAheadPoint = self.calc_lookahead_pos(car, track)
        car.lookAheadPosition = lookAheadPoint

        direction = lookAheadPoint - car.position
        angle = np.arctan2(direction[1], direction[0])

        delta_theta = angle - car.theta
        delta_theta = np.arctan2(np.sin(delta_theta), np.cos(delta_theta))

        return delta_theta