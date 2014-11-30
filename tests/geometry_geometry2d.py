"""
Tests for geometry/geometry2d.py
"""

from geometry.geometry2d import *

def test_triangle_halfspaces():
    #tri = Triangle([1,1],[2,1],[0,0])
    tri = Triangle([0,0],[1,0],[0,1])
    hspaces = tri.halfspaces
    
    fig = plt.figure()
    axes = fig.add_subplot(111)
    tri.plot(axes, 'b')
    for hspace in hspaces:
        hspace.plot(axes)
        
    plt.show(block=False)
    raw_input()

def test_triangulate():
    poly = Polygon([[0,0],[-2,-1],[-1,-2],[1,-.5]])
    triangles = poly.triangulate()
    
    fig = plt.figure()
    axes = fig.add_subplot(111)
    for tri in triangles:
        tri.plot(axes, 'r')
        
    plt.show(block=False)
    raw_input()

def test_plotting():
    fig = plt.figure()
    axes = fig.add_subplot(111)
    
    seg = Segment([0,0], [-1,2])
    seg.plot(axes, 'g')
    
    tri = Triangle([1,1],[2,1],[0,0])
    tri.plot(axes, 'r')
    
    poly = Polygon([[0,0],[-2,-1],[-1,-2],[1,-.5]])
    poly.plot(axes,'b')
    
    plt.show(block=False)
    raw_input()
        
if __name__ == '__main__':
    #test_plotting()
    #test_triangulate()
    test_triangle_halfspaces()