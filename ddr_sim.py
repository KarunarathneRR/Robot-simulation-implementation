"""
ddr_sim.py  — Differential drive robot simulator (starter)
Controls:
  1 - switch to P (distance) + PID (heading)
  2 - switch to PD (distance) + PID (heading)
  r - reset robot to start pose
  s - save trajectory CSV
  ESC - quit
Author: starter template
"""

import pygame, math, csv, time

# ---------------- params ----------------
WHEEL_BASE = 0.20       # meters (distance between wheels)
ROBOT_RADIUS = 0.08     # drawing size (m)
SCALE = 120.0           # pixels per meter
FPS = 60
MAX_V = 0.8             # m/s
MAX_OMEGA = 4.0         # rad/s

# start pose and target
start_pose = {'x': -1.0, 'y': -0.6, 'theta': math.radians(20)}
target = {'x': 1.2, 'y': 0.9}

# controller gains (tweak)
Kp_dist = 1.2
Kd_dist = 0.5   # used only in PD mode
Kp_ang = 4.0
Ki_ang = 0.0
Kd_ang = 0.4

DIST_TOL = 0.03

# ---------------- utilities ----------------
def wrap_angle(a):
    return (a + math.pi) % (2*math.pi) - math.pi

# ---------------- pygame init ----------------
pygame.init()
W, H = 1000, 700
screen = pygame.display.set_mode((W,H))
pygame.display.set_caption("DDR - P/PD + PID")
clock = pygame.time.Clock()
font = pygame.font.SysFont("consolas", 16)
center = (W//2, H//2)

def world_to_screen(x,y):
    return int(center[0] + x * SCALE), int(center[1] - y * SCALE)

# ---------------- state ----------------
pose = dict(start_pose)
path = [(pose['x'], pose['y'])]
mode = 'PD+PID'   # 'P+PID' or 'PD+PID'
prev_dist_err = None
prev_ang_err = None
ang_integral = 0.0
arrived = False
start_time = time.time()

# ---------------- main loop ----------------
running = True
while running:
    dt = clock.tick(FPS)/1000.0
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            running = False
        if ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_ESCAPE:
                running = False
            elif ev.key == pygame.K_1:
                mode = 'P+PID'
                Kd_dist = 0.0
            elif ev.key == pygame.K_2:
                mode = 'PD+PID'
                Kd_dist = 0.5
            elif ev.key == pygame.K_r:
                pose = dict(start_pose)
                path = [(pose['x'], pose['y'])]
                prev_dist_err = None
                prev_ang_err = None
                ang_integral = 0.0
                arrived = False
                start_time = time.time()
            elif ev.key == pygame.K_s:
                fn = f"trajectory_{int(time.time())}.csv"
                with open(fn, 'w', newline='') as f:
                    writer = csv.writer(f); writer.writerow(['x','y']); writer.writerows(path)
                print("Saved", fn)

    # errors
    dx = target['x'] - pose['x']
    dy = target['y'] - pose['y']
    dist_err = math.hypot(dx, dy)
    theta_d = math.atan2(dy, dx)
    ang_err = wrap_angle(theta_d - pose['theta'])

    # linear control (P or PD)
    if prev_dist_err is None:
        d_dist = 0.0
    else:
        d_dist = (dist_err - prev_dist_err)/max(dt,1e-6)
    v = Kp_dist * dist_err + Kd_dist * d_dist
    # reduce forward speed if heading error large (helps stability)
    v *= max(0.0, math.cos(ang_err))
    v = max(-MAX_V, min(MAX_V, v))

    # angular PID
    if prev_ang_err is None:
        d_ang = 0.0
    else:
        d_ang = (ang_err - prev_ang_err)/max(dt,1e-6)
    ang_integral += ang_err * dt
    # simple anti-windup
    ang_integral = max(-2.0, min(2.0, ang_integral))
    omega = Kp_ang * ang_err + Ki_ang * ang_integral + Kd_ang * d_ang
    omega = max(-MAX_OMEGA, min(MAX_OMEGA, omega))

    if dist_err < DIST_TOL:
        v = 0.0; omega = 0.0
        if not arrived:
            arrived = True
            print("Arrived in {:.2f} s".format(time.time() - start_time))

    # kinematics update (Euler)
    pose['x'] += v * math.cos(pose['theta']) * dt
    pose['y'] += v * math.sin(pose['theta']) * dt
    pose['theta'] += omega * dt
    pose['theta'] = wrap_angle(pose['theta'])

    path.append((pose['x'], pose['y']))
    prev_dist_err = dist_err
    prev_ang_err = ang_err

    # draw
    screen.fill((18,18,28))
    if len(path)>1:
        pts = [world_to_screen(x,y) for x,y in path]
        pygame.draw.lines(screen, (120,160,255), False, pts, 2)
    # target
    tx, ty = world_to_screen(target['x'], target['y'])
    pygame.draw.line(screen, (50,200,50), (tx-8,ty),(tx+8,ty),2)
    pygame.draw.line(screen, (50,200,50), (tx,ty-8),(tx,ty+8),2)
    # robot body
    sx, sy = world_to_screen(pose['x'], pose['y'])
    pygame.draw.circle(screen, (100,200,255), (sx,sy), int(ROBOT_RADIUS*SCALE), 2)
    hx = pose['x'] + 0.22*math.cos(pose['theta']); hy = pose['y'] + 0.22*math.sin(pose['theta'])
    pygame.draw.line(screen, (240,80,80), (sx,sy), world_to_screen(hx,hy), 3)

    # info text
    lines = [
      f"Mode: {mode}   (1: P+PID, 2: PD+PID, r:reset, s:save CSV)",
      f"Dist err: {dist_err:.3f} m   Heading err: {math.degrees(ang_err):.1f} deg",
      f"v={v:.3f} m/s   omega={omega:.3f} rad/s",
      f"Kp_dist={Kp_dist:.2f} Kd_dist={Kd_dist:.2f}  Kp_ang={Kp_ang:.2f} Ki_ang={Ki_ang:.2f} Kd_ang={Kd_ang:.2f}"
    ]
    for i,l in enumerate(lines):
        screen.blit(font.render(l, True, (220,220,220)), (8,8+i*18))

    pygame.display.flip()

pygame.quit()
