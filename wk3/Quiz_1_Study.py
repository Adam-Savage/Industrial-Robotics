# from spatialmath import SE2
# from math import pi   

# # Start at origin, 0 orientation
# T = SE2(0, 0, 0)
# count = 0
# for i in range(360):
#     # Translate by 1m in local X then Rotate by 1 degree (in radians)
#     T = T * SE2(1, 0, 0) * SE2(0, 0, pi/180)
#     count += 1
    
#     if count == 255:
#         print(f"After {count} transform iterations, T = \n{T}")
        



# from spatialmath import SE2
# from math import pi

# # Start poses (as given in the lab1_solution and question)
# car1_tr = SE2(300, 550, 0)
# car2_tr = SE2(130, 300, -pi/2)

# # Step transforms (GIVEN in the lab1_solution)
# RADIUS_OUTER = (550 - 66) / 2
# RADIUS_INNER = (500 - 125) / 2
# total_steps = 360
# car1_move = SE2((2 * pi * RADIUS_OUTER) / total_steps, 0, 0)
# car1_turn = SE2(0, 0, -2 * pi / total_steps)
# car2_move = SE2((2 * pi * RADIUS_INNER) / total_steps, 0, 0)
# car2_turn = SE2(0, 0, 2 * pi / total_steps)

# # Apply 23 steps to Car 1
# for _ in range(23):
#     car1_tr = car1_tr * car1_move * car1_turn

# # Apply 17 steps to Car 2
# for _ in range(17):
#     car2_tr = car2_tr * car2_move * car2_turn

# # Get Car 2's transform in Car 1's frame (code is GIVEN in the lab1_solution)
# car2_in_car1 = car1_tr.inv() * car2_tr

# # Print just the X value
# print("Car2 X in Car1 frame:", round(car2_in_car1.t[0], 2))
 







from spatialmath import SE2
from scipy import linalg

car1_tr = SE2(300, 550, 0)  # Car 1 start pose (given in solution)
car2_tr = SE2(4, 4, 0)  # Car 2 starts at [123,123]

print("dist =", linalg.norm(car1_tr.t - car2_tr.t))