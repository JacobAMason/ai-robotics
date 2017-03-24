from matplotlib import pyplot as plt
import math
import random   
import time

class Pose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
   
class Control():
    def __init__(self, linearVelocity, angularVelocity):
        self.linearVelocity = linearVelocity
        self.angularVelocity = angularVelocity
   
class VelocityParams():
    #Params 1: Influence of |linear velocity| on velocity noise
    #       2: Influence of |angular velocity| on velocity noise
    #       3: Influence of |linear velocity| on angular noise
    #       4: Influence of |angular velocity| on angular noise
    #       5: Influence of |linear velocity| on final angle noise
    #       6: Influence of |angular velocity| on final angle noise
    #       7: Delta T to be used
    def __init__(self, linearVelocityNoise, angularVelocityNoise, linearAngularNoise, angularAngularNoise, linearFinalAngleNoise, angularFinalAngleNoise, delta_t):
        self.linearVelocityNoise = linearVelocityNoise
        self.angularVelocityNoise = angularVelocityNoise
        self.linearAngularNoise = linearAngularNoise
        self.angularAngularNoise = angularAngularNoise
        self.linearFinalAngleNoise = linearFinalAngleNoise
        self.angularFinalAngleNoise = angularFinalAngleNoise
        self.delta_t = delta_t
   
def plot_robot_pose(p, c='b'):
    rradius = 0.05
    extend = 1.5
    w = 1.0
    lc = 'r'
    if c == 'r':
        lc = 'b'
    pc = plt.Circle((p.x,p.y), rradius, color=c)
    ex = p.x + extend*rradius*math.cos(p.theta)
    ey = p.y + extend*rradius*math.sin(p.theta)
    ln = plt.Line2D((p.x,ex), (p.y,ey), lw=w, color=lc)
    plt.gca().add_patch(pc)
    plt.gca().add_line(ln)
    
def sample_normal_distribution(b2):
    r = 0
    b = math.sqrt(b2)
    for i in range(12):
        r += random.uniform(-b, b)
    return r/2.0
    
def sample_normal_distribution_random(b2):
    b = math.sqrt(b2)
    return random.gauss(0, b)

def sample_triangular_distribution(b2):
    b = math.sqrt(b2)
    return (math.sqrt(6)/2)*(random.uniform(-b, b) + random.uniform(-b, b))
    
def sample_d(b):
    #return sample_triangular_distribution(b)
    #return sample_normal_distribution(b)
    return sample_normal_distribution_random(b)
    
def sample_motion_model_velocity(u, p, params):
   
    linearVelocity = u.linearVelocity + sample_d(params.linearVelocityNoise*(u.linearVelocity**2) + params.angularVelocityNoise*(u.angularVelocity**2))
    angularVelocity = u.angularVelocity + sample_d(params.linearAngularNoise*(u.linearVelocity**2) + params.angularAngularNoise*(u.angularVelocity**2))
    g = sample_d(params.linearFinalAngleNoise*(u.linearVelocity**2) + params.angularFinalAngleNoise*(u.angularVelocity**2))
    
    try:
        v_over_w = linearVelocity/angularVelocity
    except ZeroDivisionError:
        v_over_w = 0.0

    x = p.x - (v_over_w)*math.sin(p.theta) + (v_over_w)*math.sin(p.theta + angularVelocity*params.delta_t)
    y = p.y + (v_over_w)*math.cos(p.theta) - (v_over_w)*math.cos(p.theta + angularVelocity*params.delta_t)
    theta = p.theta + angularVelocity*params.delta_t + g*params.delta_t

    return Pose(x, y, theta)
    
def save_figure(fileName, u, params, samples):
    plt.clf()
    fig = plt.gcf()
    plt.axis('equal')
    plt.xlim([-2.5, 2])
    plt.ylim([-2, 2])
    
    x = Pose(-2, 0, 0)

    plot_robot_pose(x, 'r')
    for _ in range(samples):
        p = sample_motion_model_velocity(u, x, params)
        plot_robot_pose(p)

    fig.savefig(fileName)

def show_figure(fileName, u, params, samples):
    plt.clf()
    fig = plt.gcf()
    plt.axis('equal')
    plt.xlim([-2.5, 2])
    plt.ylim([-2, 2])
    
    x = Pose(-2, 0, 0)

    plot_robot_pose(x, 'r')

    begin = time.clock()

    for _ in range(samples):
        p = sample_motion_model_velocity(u, x, params)
        plot_robot_pose(p)

    end = time.clock()

    print 'It took ', end - begin, 'seconds for sampling'

    plt.show()

    
if __name__ == '__main__':
    show_figure('mmodel.jpg', Control(100, 100), VelocityParams(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.01), 500)
