from math import pi

import common
from adolphus import Scene, Camera, MultiCamera, Pose, Point, DirectionalPoint, Plane, rotation_matrix, Experiment

print "Creating camera model..."
# cameras
C = [ \
    ('T1', Pose(Point(272.0795, -512.6482, 1111.6311), rotation_matrix((-3.7496, -6.2382, -6.2694)))),
    ('T2', Pose(Point(580.7762, -27.3291, 1097.6489), rotation_matrix((-3.3748, -5.8233, -5.0194)))),
    ('T3', Pose(Point(598.8255, -532.6433, 1134.0327), rotation_matrix((-3.7631, -5.9661, -5.7504)))),
    ('T4', Pose(Point(-196.3809, -563.0857, 1130.9031), rotation_matrix((-3.7343, -0.2529, -0.6602)))),
    ('T5', Pose(Point(-233.3505, -426.0795, 1172.2064), rotation_matrix((-3.6510, -0.3514, -0.4658)))),
    ('T6', Pose(Point(126.9092, 068.0062, 1126.9477), rotation_matrix((-3.2937, -6.2415, -1.4784)))),
    ('T7', Pose(Point(101.0447, 591.6576, 1150.5325), rotation_matrix((-2.6858, -0.0903, -3.1197)))),
    ('T8', Pose(Point(559.0332, 497.1981, 1099.4856), rotation_matrix((-2.8116, -5.8239, -4.1621)))),
    ('T9', Pose(Point(-245.7283, 283.8215, 1112.6117), rotation_matrix((-2.9980, -0.3952, -1.9146)))),
    ('M1', Pose(Point(1074.0579, 169.1403, 482.5471), rotation_matrix((-3.1254, -5.1940, -4.6969)))),
    ('M2', Pose(Point(996.4342, -332.1199, 480.3496), rotation_matrix((-4.0061, -5.4570, -5.6315)))),
    ('M3', Pose(Point(848.4780, -443.7130, 547.6449), rotation_matrix((-4.0329, -5.6826, -5.9226)))),
    ('M4', Pose(Point(586.2451, -625.1676, 591.4391), rotation_matrix((-4.0677, -5.8830, -5.8559)))),
    ('M5', Pose(Point(954.5638, 488.5257, 495.3477), rotation_matrix((-2.4932, -5.3979, -3.9781)))),
    ('M6', Pose(Point(-546.4648, 120.0486, 433.7413), rotation_matrix((-3.2878, -0.9860, -1.4580)))),
    ('M7', Pose(Point(-598.3352, 143.5653, 470.7973), rotation_matrix((-3.2316, -1.0589, -1.4982)))),
    ('M8', Pose(Point(-327.1471, -248.7190, 481.7829), rotation_matrix((-3.9497, -0.5760, -0.9560)))),
    ('M9', Pose(Point(-483.0407, 602.5126, 587.3916), rotation_matrix((-2.4994, -0.7493, -2.3178)))),
    ('L1', Pose(Point(-923.6927, 225.0199, 205.0312), rotation_matrix((-3.1879, -1.4244, -1.5480)))),
    ('L2', Pose(Point(-617.1199, -250.4866, 095.83130), rotation_matrix((-4.8030, -0.9920, -0.1191)))),
    ('L3', Pose(Point(107.3623, 1440.9528, 125.8163), rotation_matrix((-1.6521, -6.2309, -3.1478)))),
    ('L4', Pose(Point(536.1135, 1104.3457, 061.0062), rotation_matrix((-1.5276, -5.8051, -3.0969)))),
    ('L5', Pose(Point(-290.4745, 752.7439, 024.6231), rotation_matrix((-1.4200, -0.9406, -3.2852)))),
    ('L6', Pose(Point(504.4358, -521.7866, 108.0059), rotation_matrix((-4.7248, -5.6686, -0.0260)))),
    ('L7', Pose(Point(034.2871, -994.4367, 134.4528), rotation_matrix((-4.6896, -0.0971, -6.2698)))),
    ('L8', Pose(Point(-286.8135, -838.0852, 135.0429), rotation_matrix((-4.7129, -0.5168, -6.2243)))),
    ]

print "Creating scene..."
# building poses
PA = Pose(Point(60, 160, 0), rotation_matrix((0, 0, 0)))
PB = Pose(Point(0, 0, 0), rotation_matrix((0, 0, 0)))
PC = Pose(Point(410, 210, 0), rotation_matrix((0, 0, 3.0 * pi / 2.0)))
PD = Pose(Point(310, 30, 0), rotation_matrix((0, 0, 3.0 * pi / 2.0)))
PE = Pose(Point(60, 290, 0), rotation_matrix((0, 0, 0)))
S = Scene()
# Floor
S.add(Plane(Pose(), (-10, 420), (-10, 380)))

def add_building_one(scene, pose, x, y, z):
    scene.add(Plane(Pose(Point(0, 0, z), None) + pose, (0, x), (0, y)))
    scene.add(Plane(Pose(None, rotation_matrix((pi / 2.0, 0, pi / 2.0))) + pose, (0, z), (0, x)))
    scene.add(Plane(Pose(Point(0, y, 0), rotation_matrix((pi / 2.0, 0, pi / 2.0))) + pose, (0, z), (0, x)))
    scene.add(Plane(Pose(None, rotation_matrix((pi / 2.0, pi / 2.0, pi / 2.0))) + pose, (0, z), (0, y)))
    scene.add(Plane(Pose(Point(x, 0, 0), rotation_matrix((pi / 2.0, pi / 2.0, pi / 2.0))) + pose, (0, z), (0, y)))

def add_building_two(scene, pose, x, y, z):
    scene.add(Plane(Pose(Point(0, 0, z) + pose.T, None), (-x, 0), (0, y)))
    scene.add(Plane(Pose(None, rotation_matrix((pi / 2.0, 0, pi / 2.0))) + pose, (0, z), (0, y)))
    scene.add(Plane(Pose(Point(0, x, 0), rotation_matrix((pi / 2.0, 0, pi / 2.0))) + pose, (0, z), (0, y)))
    scene.add(Plane(Pose(None, rotation_matrix((pi / 2.0, pi / 2.0, pi / 2.0))) + pose, (0, z), (0, x)))
    scene.add(Plane(Pose(Point(y, 0, 0), rotation_matrix((pi / 2.0, pi / 2.0, pi / 2.0))) + pose, (0, z), (0, x)))

# A
add_building_one(S, PA, 100, 100, 200)
# B
add_building_one(S, PB, 100, 90, 300)
# C
add_building_two(S, PC, 200, 150, 200)
# D
add_building_two(S, PD, 170, 90, 140)
# E
add_building_one(S, PE, 120, 80, 100)

print "Creating points..."
P = []
# A
P.append(PA.map(DirectionalPoint(30, 40, 200, 0, 0)))
P.append(PA.map(DirectionalPoint(50, 80, 200, 0, 0)))
P.append(PA.map(DirectionalPoint(0, 50, 60, pi / 2.0, pi)))
P.append(PA.map(DirectionalPoint(0, 80, 150, pi / 2.0, pi)))
P.append(PA.map(DirectionalPoint(30, 0, 100, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PA.map(DirectionalPoint(70, 0, 160, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PA.map(DirectionalPoint(90, 0, 20, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PA.map(DirectionalPoint(100, 30, 60, pi / 2.0, 0)))
P.append(PA.map(DirectionalPoint(100, 50, 120, pi / 2.0, 0)))
P.append(PA.map(DirectionalPoint(10, 100, 90, pi / 2.0, pi / 2.0)))
P.append(PA.map(DirectionalPoint(70, 100, 60, pi / 2.0, pi / 2.0)))
P.append(PA.map(DirectionalPoint(70, 100, 190, pi / 2.0, pi / 2.0)))
# B
P.append(PB.map(DirectionalPoint(50, 0, 70, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PB.map(DirectionalPoint(80, 0, 150, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PB.map(DirectionalPoint(40, 0, 230, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PB.map(DirectionalPoint(100, 20, 270, pi / 2.0, 0)))
P.append(PB.map(DirectionalPoint(100, 10, 100, pi / 2.0, 0)))
P.append(PB.map(DirectionalPoint(100, 60, 100, pi / 2.0, 0)))
P.append(PB.map(DirectionalPoint(20, 100, 40, pi / 2.0, pi / 2.0)))
P.append(PB.map(DirectionalPoint(70, 100, 140, pi / 2.0, pi / 2.0)))
P.append(PB.map(DirectionalPoint(60, 100, 210, pi / 2.0, pi / 2.0)))
P.append(PB.map(DirectionalPoint(0, 50, 100, pi / 2.0, pi)))
P.append(PB.map(DirectionalPoint(0, 20, 200, pi / 2.0, pi)))
P.append(PB.map(DirectionalPoint(0, 30, 240, pi / 2.0, pi)))
# C
P.append(PC.map(DirectionalPoint(40, 30, 200, 0, 0)))
P.append(PC.map(DirectionalPoint(90, 70, 200, 0, 0)))
P.append(PC.map(DirectionalPoint(80, 140, 200, 0, 0)))
P.append(PC.map(DirectionalPoint(10, 180, 200, 0, 0)))
P.append(PC.map(DirectionalPoint(50, 0, 50, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PC.map(DirectionalPoint(110, 0, 150, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PC.map(DirectionalPoint(100, 160, 150, pi / 2.0, 0)))
P.append(PC.map(DirectionalPoint(100, 60, 90, pi / 2.0, 0)))
P.append(PC.map(DirectionalPoint(100, 150, 90, pi / 2.0, 0)))
P.append(PC.map(DirectionalPoint(100, 100, 20, pi / 2.0, 0)))
P.append(PC.map(DirectionalPoint(80, 200, 40, pi / 2.0, pi / 2.0)))
P.append(PC.map(DirectionalPoint(50, 200, 140, pi / 2.0, pi / 2.0)))
P.append(PC.map(DirectionalPoint(0, 140, 90, pi / 2.0, pi)))
P.append(PC.map(DirectionalPoint(0, 50, 30, pi / 2.0, pi)))
P.append(PC.map(DirectionalPoint(0, 40, 130, pi / 2.0, pi)))
#D
P.append(PD.map(DirectionalPoint(80, 0, 20, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PD.map(DirectionalPoint(40, 0, 90, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PD.map(DirectionalPoint(90, 50, 100, pi / 2.0, 0)))
P.append(PD.map(DirectionalPoint(90, 100, 40, pi / 2.0, 0)))
P.append(PD.map(DirectionalPoint(50, 170, 40, pi / 2.0, pi / 2.0)))
P.append(PD.map(DirectionalPoint(40, 170, 110, pi / 2.0, pi / 2.0)))
P.append(PD.map(DirectionalPoint(0, 30, 60, pi / 2.0, pi)))
P.append(PD.map(DirectionalPoint(0, 120, 60, pi / 2.0, pi)))
P.append(PD.map(DirectionalPoint(40, 80, 140, 0, 0)))
#E
P.append(PE.map(DirectionalPoint(30, 0, 60, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PE.map(DirectionalPoint(90, 0, 70, pi / 2.0, 3.0 * pi / 2.0)))
P.append(PE.map(DirectionalPoint(120, 40, 50, pi / 2.0, 0)))
P.append(PE.map(DirectionalPoint(40, 80, 80, pi / 2.0, pi / 2.0)))
P.append(PE.map(DirectionalPoint(70, 80, 80, pi / 2.0, pi / 2.0)))
P.append(PE.map(DirectionalPoint(0, 50, 30, pi / 2.0, pi)))
P.append(PE.map(DirectionalPoint(110, 70, 100, 0, 0)))
P.append(PE.map(DirectionalPoint(50, 30, 100, 0, 0)))

print "Creating discrete multi-camera model..."
M = MultiCamera(ocular = 2, scene = S, points = P)
for name, pose in C:
    M[name] = Camera(4.4765, 12.4922, 0.00465, (691.1516, 500.7902), (1360, 1024), 1216.1, 20, 3.0, 0.5, 0.036, 0.3, pose = pose)

print "Running experiment..."
E = Experiment(M)
E.run()
