import random
import math
import time

from DWMBot import DWMBot
from eTaxi_Simulated import eTaxi_Simulated
from navigation_control import drive_to_target, get_line_angle, set_plane, navigate_bot, get_target
# from visualization import make_plot

MAX_NUM_TRIALS = 1000
MAX_NUM_STEPS = 90000


def simulated_navigation_test(bulk_test=False, num_trials=MAX_NUM_TRIALS):
    global target_x_pos, target_y_pos, start_y_pos, start_x_pos
    # new_x = 10.2
    # new_y = 10.2
    # target_x_pos = 2000.2
    # target_y_pos = 3500.2
    # set_true_position(new_x, new_y)
    # drive_to_target(MAX_NUM_STEPS)

    # new_x = 5500.2
    # new_y = 4000.2
    # target_x_pos = 100.2
    # target_y_pos = 110.3
    # set_true_position(new_x, new_y)
    # drive_to_target(MAX_NUM_STEPS)

    # new_x = 10.2
    # new_y = 2000.2
    # target_x_pos = 2500.2
    # target_y_pos = 10.2
    # set_true_position(new_x, new_y)
    # drive_to_target(MAX_NUM_STEPS)
    #
    # new_x = 2000.2
    # new_y = 10.2
    # target_x_pos = 10.2
    # target_y_pos = 3500.2
    # set_true_position(new_x, new_y)
    # drive_to_target(MAX_NUM_STEPS)
    eTaxi = eTaxi_Simulated()
    if bulk_test:
        num_failures = 0
        for x in range(num_trials):
            if x % 100000 == 0:
                print("Trial: ", x)
            new_x = random.randint(0, 10000)
            new_y = random.randint(0, 10000)
            eTaxi.set_true_position(new_x, new_y)
            inital_x = new_x
            inital_y = new_y
            plane_x = random.randint(0, 10000)
            plane_y = random.randint(0, 10000)
            plane_heading = (random.randint(0, 100)/100) * 2*math.pi

            set_plane(plane_x, plane_y, plane_heading)
            step_count, rec_x, rec_y, adj_x, adj_y, measured_x, measured_y, defined_start_x, defined_start_y = drive_to_target(eTaxi, MAX_NUM_STEPS, bulk_test=True)
            if step_count >= MAX_NUM_STEPS:
                num_failures += 1
                print('start_coords: ', inital_x, inital_y)
                print('bot start coords: ', defined_start_x, defined_start_y)
                target_x, target_y = get_target()
                print('target_coords', target_x, target_y)
                print('end_coords: ', eTaxi.get_true_position())
                line_angle = math.degrees(get_line_angle())
                print('line_angle: ', line_angle)
                print()
                # make_plot(rec_x, rec_y, adj_x, adj_y, measured_x, measured_y, target_x, target_y, inital_x, inital_y,
                #           plane_x=plane_x, plane_y=plane_y)

        print('Results:')
        print('Trials Run: ', num_trials)
        print("Trials Failed:  ", num_failures)
        print('Failure Rate: ', num_failures/num_trials)


def test_drive_to_point():
    print('in test drive to point!')
    eTaxi = DWMBot()
    print('initialized eTaxi')

    time.sleep(0.5)
    for _ in range(20):
        inital_x, inital_y = eTaxi.get_position()
        print('starting_position: ', inital_x, inital_y)
    plane_x = 215
    plane_y = 150
    plane_heading = math.pi - (math.pi/9)
    set_plane(plane_x, plane_y, plane_heading)
    print('plane position set')
    target_x, target_y = get_target()
    print('target_pos: ', target_x, target_y)
    step_count, rec_x, rec_y, adj_x, adj_y, measured_x, measured_y, defined_start_x, defined_start_y = drive_to_target(
        eTaxi, MAX_NUM_STEPS, bulk_test=True)

    final_x, final_y = eTaxi.get_position()
    print('final position: ', final_x, final_y)
    eTaxi.shut_down()
    # make_plot(rec_x, rec_y, adj_x, adj_y, measured_x, measured_y, target_x, target_y, inital_x, inital_y,
    #           plane_x=plane_x, plane_y=plane_y)


def test_drive_and_acquire():
    eTaxi = eTaxi_Simulated()
    for x in range(1):
        new_x = random.randint(0, 10000)
        new_y = random.randint(0, 10000)
        # new_x = 20
        # new_y = 20
        eTaxi.set_true_position(new_x, new_y)
        inital_x = new_x
        inital_y = new_y
        # plane_x = 10000
        # plane_y = 10000
        # plane_heading = 4.41568
        plane_x = random.randint(0, 10000)
        plane_y = random.randint(0, 10000)
        plane_heading = (random.randint(0, 100) / 100) * 2 * math.pi
        set_plane(plane_x, plane_y, plane_heading)
        step_count, rec_x, rec_y, adj_x, adj_y, measured_x, measured_y, defined_start_x, defined_start_y = drive_to_target(
            eTaxi, MAX_NUM_STEPS, bulk_test=True)
        # dock(plane_x, plane_y, plane_heading)
        target_x, target_y = get_target()

        # make_plot(rec_x, rec_y, adj_x, adj_y, measured_x, measured_y, target_x, target_y, inital_x, inital_y,
        #           plane_x=plane_x, plane_y=plane_y)



def demo_waypoints():
    eTaxi = DWMBot()

    way_points = []
    point_1 = [160, 160]
    point_2 = [215, 160]
    point_3 = [215, 330]
    point_4 = [160, 160]

    way_points.append(point_1)
    way_points.append(point_2)
    way_points.append(point_3)
    way_points.append(point_4)

    navigate_bot(eTaxi, way_points)

    eTaxi.shut_down()



# test_drive_to_point()

demo_waypoints()
