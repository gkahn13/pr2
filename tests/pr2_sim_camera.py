"""
Tests for pr2_sim/camera.py
"""

from pr2_sim.camera import *

def test_signed_distance():
    sim = simulator.Simulator(view=True)
    larm = arm.Arm('left',sim=sim)
    larm.set_posture('mantis')
    rarm = arm.Arm('right',sim=sim)
    rarm.set_posture('mantis')
    
    cam = Camera(rarm, sim)
    #     triangles3d = [geometry3d.Triangle([.7,0,.8],[.7,0,1.1],[.7,-.3,.7])]
#     triangles3d = [geometry3d.Triangle([.5,0,.5],[.8,0,.6],[.5,-.3,.9])]
#     triangles3d = [geometry3d.Triangle([np.random.uniform(.2,.5), np.random.uniform(-.5,0), np.random.uniform(.25,.75)],
#                                        [np.random.uniform(.2,.5), np.random.uniform(-.5,0), np.random.uniform(.25,.75)],
#                                        [np.random.uniform(.2,.5), np.random.uniform(-.5,0), np.random.uniform(.25,.75)]) for _ in xrange(3)]
    table_center = np.array([.2,.7,.5])
    triangles3d = [#geometry3d.Triangle(table_center, table_center+np.array([.5,-1.4,0]), table_center+np.array([.5,0,0])),
                   #geometry3d.Triangle(table_center, table_center+np.array([0,-1.4,0]), table_center+np.array([.5,-1.4,0])),
                   geometry3d.Triangle(table_center+np.array([.25,-.7,0]), table_center+np.array([.25,-.7,.2]), table_center+np.array([.25,-1.2,0])),
                   geometry3d.Triangle(table_center+np.array([.25,-1.2,0]), table_center+np.array([.25,-.7,.2]), table_center+np.array([.25,-1.2,.2]))]
    truncated_frustum = cam.truncated_view_frustum(triangles3d, plot=True)
    
    pos_step = .01
    delta_position = {'a' : [0, pos_step, 0],
                      'd' : [0, -pos_step, 0],
                      'w' : [pos_step, 0, 0],
                      'x' : [-pos_step, 0, 0],
                      '+' : [0, 0, pos_step],
                      '-' : [0, 0, -pos_step]}
    point = tfx.point(table_center)
    
    print('Move point around with keyboard to test signed-distance')
    char = ''
    while char != 'q':
        char = utils.Getch.getch()
        
        sim.clear_plots(1)
        if delta_position.has_key(char):
            point += delta_position[char]
        sim.plot_point(sim.transform_from_to(point.array, 'base_link', 'world'), size=.02, color=(0,0,1))
        
        sd = cam.signed_distance(point, truncated_frustum)
        print('sd: {0}'.format(sd))

    print('point: {0}'.format(point))

def test_truncated_view_frustum():
    sim = simulator.Simulator(view=True)
    larm = arm.Arm('left',sim=sim)
    larm.set_posture('mantis')
    rarm = arm.Arm('right',sim=sim)
    rarm.set_posture('mantis')
    
    cam = Camera(rarm, sim)
#     triangles3d = [geometry3d.Triangle([.7,0,.8],[.7,0,1.1],[.7,-.3,.7])]
#     triangles3d = [geometry3d.Triangle([.5,0,.5],[.8,0,.6],[.5,-.3,.9])]
#     triangles3d = [geometry3d.Triangle([np.random.uniform(.2,1), np.random.uniform(-.5,0), np.random.uniform(.25,.75)],
#                                        [np.random.uniform(.2,1), np.random.uniform(-.5,0), np.random.uniform(.25,.75)],
#                                        [np.random.uniform(.2,1), np.random.uniform(-.5,0), np.random.uniform(.25,.75)]) for _ in xrange(3)]
#     table_center = np.array([.2,.7,.5])
#     triangles3d = [geometry3d.Triangle(table_center, table_center+np.array([.5,-1.4,0]), table_center+np.array([.5,0,0])),
#                    geometry3d.Triangle(table_center, table_center+np.array([0,-1.4,0]), table_center+np.array([.5,-1.4,0])),
#                    geometry3d.Triangle(table_center+np.array([.25,-.7,0]), table_center+np.array([.25,-.7,.2]), table_center+np.array([.25,-.9,0]))]
    
#     triangles3d = list()
#     triangles3d.append(geometry3d.Triangle([.5,-.5,.55], [.5,-.55,.55], [.5,-.55,.65]))
#     triangles3d.append(geometry3d.Triangle([.5,-.5,.55], [.5,-.5,.65], [.5,-.55,.65]))
#     # table
#     triangles3d.append(geometry3d.Triangle([0,-1,.55], [0,1,.55], [1,1,.55]))
#     triangles3d.append(geometry3d.Triangle([0,-1,.55], [1,-1,.55], [1,1,.55]))

    triangles3d = [geometry3d.Triangle([0,2,.45], [0,-2,.45], [5,0,.45]),
            geometry3d.Triangle([0.537954, -0.334495,  0.548002], [0.497772, -0.351674,  0.458056], [0.525739, -0.392862, 0.45343]),
            geometry3d.Triangle([0.537954, -0.334495,  0.548002], [0.565921, -0.375683,  0.543376], [0.525739, -0.392862,   0.45343]),
            geometry3d.Triangle([0.426088, -0.169742,  0.566509], [0.385906, -0.186921,  0.476563], [0.35794, -0.145733,   0.48119]),
            geometry3d.Triangle([0.426088, -0.169742,  0.566509], [0.398122, -0.128553,  0.571136], [0.35794, -0.145733,   0.48119])]
    
#     for tri3d in triangles3d:
#         tri3d.plot(sim, color=(0,0,1), fill=True, alpha=0.25)
#     
#     rarm.teleop()
#     sim.clear_plots()
    
    cam.truncated_view_frustum(triangles3d, plot=True)
    
    print('Press enter to exit')
    raw_input()
    

def test_project_triangles():
    sim = simulator.Simulator(view=True)
    rarm = arm.Arm('right',sim=sim)
    rarm.set_posture('mantis')
    
    cam = Camera(rarm, sim)
    triangles3d = [geometry3d.Triangle([.7,0,.8],[.7,0,1.1],[.7,-.3,.7])]
#     triangles3d = [geometry3d.Triangle(cam.segment_through_pixel([1,1]).p1,
#                                        cam.segment_through_pixel([cam.height-1,cam.width-1]).p1,
#                                        cam.segment_through_pixel([1,cam.width-1]).p1)]
    triangles2d = cam.project_triangles(triangles3d)
    
    for triangle3d in triangles3d:
        triangle3d.plot(sim)
    cam.plot()
    
    fig = plt.figure()
    axes = fig.add_subplot(111)
    
    for t in triangles2d:
        a, b, c = t.a, t.b, t.c
        a_flip = [a[1], cam.height - a[0]]
        b_flip = [b[1], cam.height - b[0]]
        c_flip = [c[1], cam.height - c[0]]
        geometry2d.Triangle(a_flip,b_flip,c_flip).plot(axes, color='b')
    
    plt.xlim((-1, cam.width+1))
    plt.ylim((-1, cam.height+1))
        
    plt.show(block=False)
        
    print('Press enter to exit')
    raw_input()
    

def test_camera_teleop():
    sim = simulator.Simulator(view=True)
    rarm = arm.Arm('right',sim=sim)
    rarm.set_posture('mantis')
    
    cam = Camera(rarm, sim)
    
    rarm.teleop()
    cam.plot(sim)
    
    print('Press enter to exit')
    raw_input()

if __name__ == '__main__':
    #test_signed_distance()
    test_truncated_view_frustum()
    #test_project_triangles()
    #test_camera_teleop()
    