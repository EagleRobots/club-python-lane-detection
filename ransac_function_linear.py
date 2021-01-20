# Performs RANSAC calculations for inputted potential points
# Referenced by "ransac_warp_lane_detection.py" script
# Referenced by "ransac_trajectory.py" script
import random
from math import sqrt
import itertools

def compute_ransac(input_points, max_distance, max_iterations, ratio):
    output = []
    old_count = 0
    iterations = 0
    # remove duplicate points to prevent useless iterations where the same point is selected twice
    input_points.sort()
    input_points = list(input_points for input_points,_ in itertools.groupby(input_points))

    total_points = len(input_points)
#    print("Number of input points: {}".format(total_points))

    # if we don't have at least two input points to test, then the algorithm cannot be run
    if total_points < 2:
        print("Not enough inputs")
    
    else:
        min_inliers = int(total_points * ratio)

        # Perform RANSAC calculations up to the maximum iteration limit if necessary
        for _ in range(max_iterations):
            # Call the ransacing function and increment the iteration count
            [x_coord, y_coord] = compute_single_round(input_points, max_distance)
            iterations+=1
#             print("Iteration #: {}".format(iterations))
#             print("Inliers: {}".format(len(x_coord)))
#             print("Minimum: {}".format(min_inliers))
            
            # If a sufficient number of inliers have been found, return that list
            if len(x_coord) > min_inliers:
                return [x_coord, y_coord]

def compute_single_round(input_points, max_distance):

    # choose two random points to sample
    chosen_points = random.sample(input_points, 2)
    # make sure we actually copy the list
    input_points_filtered = input_points[:]
    # remove the points we are using from this iteration of the loop
    input_points_filtered.remove(chosen_points[0])
    input_points_filtered.remove(chosen_points[1])

    x_inliers = []
    y_inliers = []

    outliers = input_points_filtered[:]

    # numerator and denominator of the slope
    bottom = chosen_points[1][0] - chosen_points[0][0]
    top = chosen_points[1][1] - chosen_points[0][1]

    if bottom == 0:
        slope = None
        y_int = None
    elif top == 0:
        slope = 0
        y_int = chosen_points[0][0]
    else:
        slope = top / (1.0 * bottom)
        y_int = chosen_points[0][1] - (chosen_points[0][0] * slope)

    # now that we have a slope and y intercept we can start testing the other points to find inliers
    for point in input_points_filtered:
        # if the line is vertical, use just y component for distance
        if slope == None:
            distance = abs(chosen_points[0][1] - point[1])
        # if the line is horiozontal, use just x component for distance
        elif slope == 0:
            distance = abs(chosen_points[0][0] - point[0])
        # otherwise compute the line from the outlier point to the model line, and then get the point of intersection on the model line
        # then compute the distance and check if it is acceptable
        else:
            slope_normal = -1/slope
            x = (point[1] - y_int - (slope_normal * point[0])) / (slope - slope_normal)
            y = slope_normal * (x - point[0]) + point[1]
            distance = sqrt( (x - point[0]) ** 2 + (y - point[1]) ** 2 )
        if distance <= max_distance:
            x_inliers.append(point[0])
            y_inliers.append(point[1])
            outliers.remove(point)

    return [x_inliers, y_inliers]

# Run the functions in main
if __name__ == "__main__":

    # if we call this script directly explain that there is a separate demo file
    print("To use this file, simply import it.  To see a demo of how line fitting RANSAC works, run the linear_demo.py file located in this directory.")
