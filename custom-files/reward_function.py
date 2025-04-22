import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-2.03192145, -5.93320933],
       [-1.73802011, -5.98174966],
       [-1.44381687, -6.02622622],
       [-1.1496732 , -6.06658437],
       [-0.85589123, -6.10278582],
       [-0.56272693, -6.13469799],
       [-0.27040043, -6.16214232],
       [ 0.02084131, -6.18467276],
       [ 0.31072383, -6.20167845],
       [ 0.59898707, -6.21256043],
       [ 0.88520639, -6.21611225],
       [ 1.16889281, -6.21098417],
       [ 1.44941323, -6.19548748],
       [ 1.7258983 , -6.16745928],
       [ 1.99709839, -6.12409579],
       [ 2.26118535, -6.0618842 ],
       [ 2.51481566, -5.97549424],
       [ 2.75259277, -5.85974869],
       [ 2.97600119, -5.72254771],
       [ 3.18544319, -5.56732541],
       [ 3.38165608, -5.3969202 ],
       [ 3.5653417 , -5.21345187],
       [ 3.73701265, -5.01843984],
       [ 3.89711226, -4.81307528],
       [ 4.04538624, -4.59781231],
       [ 4.18117693, -4.37279873],
       [ 4.30325813, -4.13785302],
       [ 4.40732684, -3.89116054],
       [ 4.49421704, -3.63464103],
       [ 4.56920257, -3.37198328],
       [ 4.63393935, -3.10440323],
       [ 4.68943508, -2.83267512],
       [ 4.73673986, -2.5575696 ],
       [ 4.77631171, -2.2805938 ],
       [ 4.80749867, -2.013134  ],
       [ 4.84500784, -1.7505322 ],
       [ 4.88994526, -1.49195586],
       [ 4.94334141, -1.23803289],
       [ 5.00664533, -0.98965356],
       [ 5.08131685, -0.74775629],
       [ 5.17079824, -0.51451001],
       [ 5.27912841, -0.29263789],
       [ 5.40212831, -0.0798514 ],
       [ 5.5370311 ,  0.12553722],
       [ 5.6815107 ,  0.32517795],
       [ 5.83417797,  0.52035615],
       [ 5.99376943,  0.71235397],
       [ 6.15900824,  0.90229628],
       [ 6.32847621,  1.0910584 ],
       [ 6.50098472,  1.27918543],
       [ 6.68236229,  1.4747822 ],
       [ 6.85925113,  1.67249856],
       [ 7.02716229,  1.87437613],
       [ 7.18173052,  2.08221762],
       [ 7.3181415 ,  2.29769395],
       [ 7.4303757 ,  2.52235201],
       [ 7.51118571,  2.75690038],
       [ 7.55339079,  2.99969747],
       [ 7.55655498,  3.24598534],
       [ 7.52488597,  3.49106536],
       [ 7.46367567,  3.73175593],
       [ 7.37782503,  3.96627543],
       [ 7.27099175,  4.19362208],
       [ 7.14345995,  4.41189969],
       [ 6.9981869 ,  4.62055863],
       [ 6.83766546,  4.81946479],
       [ 6.66317509,  5.00808304],
       [ 6.47593649,  5.1860127 ],
       [ 6.27701616,  5.35289446],
       [ 6.06695635,  5.50790467],
       [ 5.84610603,  5.64984264],
       [ 5.61249231,  5.7731663 ],
       [ 5.36749821,  5.8747253 ],
       [ 5.11499899,  5.95716393],
       [ 4.85736125,  6.02278201],
       [ 4.59615682,  6.07367086],
       [ 4.33231709,  6.11088342],
       [ 4.06654327,  6.13528425],
       [ 3.79937702,  6.14755717],
       [ 3.53125321,  6.14819555],
       [ 3.26254904,  6.13764188],
       [ 2.99359923,  6.11533441],
       [ 2.72482016,  6.08022191],
       [ 2.45676803,  6.03098154],
       [ 2.19045122,  5.9642007 ],
       [ 1.92786429,  5.87513902],
       [ 1.66893611,  5.77008746],
       [ 1.41316516,  5.65287578],
       [ 1.15992124,  5.52644102],
       [ 0.90855988,  5.39326379],
       [ 0.65843641,  5.25567009],
       [ 0.40890319,  5.1159699 ],
       [ 0.1531643 ,  4.97507223],
       [-0.10382713,  4.83629333],
       [-0.36229042,  4.70016283],
       [-0.62242837,  4.56722302],
       [-0.88448726,  4.43815122],
       [-1.14870308,  4.31366372],
       [-1.41519026,  4.19425665],
       [-1.6839671 ,  4.08021453],
       [-1.95496881,  3.97161015],
       [-2.22805019,  3.86829745],
       [-2.50299863,  3.76993336],
       [-2.77954536,  3.67598691],
       [-3.05737164,  3.5857278 ],
       [-3.33615145,  3.49832718],
       [-3.61567306,  3.4132375 ],
       [-3.89574814,  3.32994128],
       [-4.16811288,  3.25058644],
       [-4.43939869,  3.16980977],
       [-4.70842339,  3.08598323],
       [-4.97411768,  2.99749066],
       [-5.23543338,  2.90260653],
       [-5.49124003,  2.79947027],
       [-5.7400496 ,  2.68587554],
       [-5.97995977,  2.55935369],
       [-6.20828062,  2.41682423],
       [-6.42096974,  2.25441735],
       [-6.61137768,  2.0676342 ],
       [-6.78127982,  1.86210423],
       [-6.93306174,  1.64233896],
       [-7.06802269,  1.41108482],
       [-7.18722699,  1.17049199],
       [-7.29150203,  0.92226917],
       [-7.38106998,  0.66766077],
       [-7.45672149,  0.40800814],
       [-7.5189618 ,  0.14435615],
       [-7.56764392, -0.12250307],
       [-7.60263224, -0.39175306],
       [-7.62377264, -0.66256247],
       [-7.6308811 , -0.93408301],
       [-7.62380904, -1.20545056],
       [-7.60252693, -1.47580721],
       [-7.56720649, -1.74435376],
       [-7.51843016, -2.01044829],
       [-7.45738608, -2.27371616],
       [-7.38520654, -2.53393729],
       [-7.30143861, -2.79049008],
       [-7.20548935, -3.04260749],
       [-7.09559435, -3.28886581],
       [-6.96929377, -3.52700345],
       [-6.82869788, -3.75695194],
       [-6.67165231, -3.97622417],
       [-6.49509733, -4.1808456 ],
       [-6.29870237, -4.36795472],
       [-6.08539141, -4.53774773],
       [-5.8586835 , -4.69228033],
       [-5.62102591, -4.83337074],
       [-5.37427696, -4.96262901],
       [-5.11989091, -5.0814621 ],
       [-4.85891187, -5.1909223 ],
       [-4.59233225, -5.2921354 ],
       [-4.32080427, -5.38582848],
       [-4.04491019, -5.47267934],
       [-3.76517645, -5.55331527],
       [-3.48208663, -5.62830691],
       [-3.19609531, -5.6981582 ],
       [-2.90764314, -5.76329461],
       [-2.61716912, -5.82404939],
       [-2.32511485, -5.880649  ],
       [-2.03192145, -5.93320933]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)