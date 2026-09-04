# laptop_client_impulse.py
import socket, pygame, math, time

HOST = "10.0.0.124"   # Replace with your RDK’s IP
PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

pygame.init()
screen = pygame.display.set_mode((600, 600))
pygame.display.set_caption("DFRobot IMU Virtual Area")
clock = pygame.time.Clock()

cube_vertices = [[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                 [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]]
cube_edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]

def rotate_point(x,y,z,yaw,pitch,roll):
    yaw, pitch, roll = -math.radians(yaw), -math.radians(pitch), -math.radians(roll)
    # Pitch
    x1=x; y1=y*math.cos(pitch)-z*math.sin(pitch); z1=y*math.sin(pitch)+z*math.cos(pitch)
    # Roll
    x2=x1*math.cos(roll)+z1*math.sin(roll); y2=y1; z2=-x1*math.sin(roll)+z1*math.cos(roll)
    # Yaw
    x3=x2*math.cos(yaw)-y2*math.sin(yaw); y3=x2*math.sin(yaw)+y2*math.cos(yaw); z3=z2
    return x3,y3,z3

def project_point(x,y,z,tx,ty,scale=100):
    factor=scale/(z+5)
    return int(x*factor+300+tx), int(y*factor+300+ty)

yaw=pitch=roll=0; ax=ay=az=0
tx=ty=0

running=True
while running:
    for event in pygame.event.get():
        if event.type==pygame.QUIT: running=False
        elif event.type==pygame.KEYDOWN and event.key==pygame.K_r:
            tx=ty=0

    try:
        data=sock.recv(1024).decode('utf-8').strip()
        if data:
            for line in data.splitlines():
                parts=line.split(',')
                if len(parts)>=9:
                    ax,ay,az,gx,gy,gz,yaw,pitch,roll=[float(v) for v in parts]
    except: pass

    # Gravity compensation
    g=9.81
    gx_comp=-math.sin(math.radians(pitch))*g
    gy_comp= math.sin(math.radians(roll))*g
    gz_comp= math.cos(math.radians(pitch))*math.cos(math.radians(roll))*g
    ax_lin=ax-gx_comp; ay_lin=ay-gy_comp; az_lin=az-gz_comp

    # Threshold filter: ignore tiny noise
    threshold=0.3
    if abs(ax_lin)>threshold: ty += -ax_lin*5   # IMU X forward → screen Y
    if abs(ay_lin)>threshold: tx += ay_lin*5    # IMU Y left → screen X

    # Decay back to center slowly (prevents drift)
    tx *= 0.95
    ty *= 0.95

    # Draw cube
    screen.fill((0,0,0))
    transformed=[project_point(*rotate_point(*v,yaw,pitch,roll),tx,ty) for v in cube_vertices]
    for e in cube_edges:
        pygame.draw.line(screen,(0,255,0),transformed[e[0]],transformed[e[1]],2)
    pygame.display.flip(); clock.tick(60)

pygame.quit()
