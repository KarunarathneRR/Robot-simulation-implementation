# main.py (fixed logging + proper controllers)
import pygame, sys, math, time, csv
from robot import Robot, wrap_angle
from controllers import PController, PDController, PIDController

# ----------------------------
# Simulation Parameters
# ----------------------------
SCREEN_W, SCREEN_H = 900, 600
SCALE, CENTER = 100.0, (SCREEN_W//2, SCREEN_H//2)
MAX_V, MAX_OMEGA = 0.6, 3.0
TARGET = (1.0, 0.8)   # single fixed target
STOP_DIST, STOP_ANGLE = 0.05, 0.1

def world_to_screen(x,y):
    return (CENTER[0]+int(x*SCALE), CENTER[1]-int(y*SCALE))

def compute_errors(robot, target):
    dx, dy = target[0]-robot.x, target[1]-robot.y
    dist = math.hypot(dx, dy)
    desired = math.atan2(dy, dx)
    return dist, wrap_angle(desired - robot.theta)

# ----------------------------
# Initialization
# ----------------------------
pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 16)

robot = Robot(x=-1.0, y=-0.5, theta=0.0)
trajectory, log = [], []
mode, stopped = "P-only", False
start_time = time.time()

# Default = P-only controllers
lin_ctrl = PController(kp=0.8, output_limit=MAX_V)
head_ctrl = PController(kp=3.0, output_limit=MAX_OMEGA)

print("Press 1 = P-only, Press 2 = PD+PID")

# ----------------------------
# Main Simulation Loop
# ----------------------------
running = True
while running:
    dt = clock.tick(60) / 1000.0
    for e in pygame.event.get():
        if e.type == pygame.QUIT: running = False
        elif e.type == pygame.KEYDOWN:
            if e.key == pygame.K_1:
                lin_ctrl = PController(kp=0.8, output_limit=MAX_V)
                head_ctrl = PController(kp=3.0, output_limit=MAX_OMEGA)
                mode, stopped = "P-only", False
                robot = Robot(x=-1.0, y=-0.5, theta=0.0)
                trajectory.clear()
                log.clear()
                start_time = time.time()
                print("Switched to P-only controller")
            elif e.key == pygame.K_2:
                lin_ctrl = PDController(kp=0.8, kd=0.2, output_limit=MAX_V)
                head_ctrl = PIDController(kp=4.0, ki=0.05, kd=0.2,
                                          output_limit=MAX_OMEGA, integrator_limit=1.0)
                mode, stopped = "PD+PID", False
                robot = Robot(x=-1.0, y=-0.5, theta=0.0)
                trajectory.clear()
                log.clear()
                start_time = time.time()
                print("Switched to PD+PID controller")

    if not stopped:
        dist, heading_err = compute_errors(robot, TARGET)

        # Use correct update method
        if isinstance(lin_ctrl, (PDController, PIDController)):
            v_cmd = lin_ctrl.update(dist, dt)
        else:
            v_cmd = lin_ctrl.update(dist)

        if isinstance(head_ctrl, (PDController, PIDController)):
            omega_cmd = head_ctrl.update(heading_err, dt)
        else:
            omega_cmd = head_ctrl.update(heading_err)

        # Clamp values
        v_cmd = max(min(v_cmd, MAX_V), -MAX_V)
        omega_cmd = max(min(omega_cmd, MAX_OMEGA), -MAX_OMEGA)

        # Update robot kinematics
        v_r, v_l = v_cmd + omega_cmd*robot.wheel_base/2.0, v_cmd - omega_cmd*robot.wheel_base/2.0
        w_r, w_l = v_r/robot.wheel_radius, v_l/robot.wheel_radius
        robot.update_from_wheel_angular(w_l, w_r, dt)

        # Save trajectory & log
        trajectory.append((robot.x, robot.y))
        log.append((time.time()-start_time, robot.x, robot.y, robot.theta, dist, heading_err, v_cmd, omega_cmd))

        # Stopping condition
        if dist < STOP_DIST and abs(heading_err) < STOP_ANGLE:
            stopped = True
            # Save screenshot
            fname = f"final_pose_{mode}.png"
            pygame.image.save(screen, fname)
            print(f"{mode} reached target. Screenshot saved as {fname}")
            # Save log with mode in filename
            with open(f"sim_log_{mode.replace('+','_')}.csv","w",newline="") as f:
                writer=csv.writer(f)
                writer.writerow(["t","x","y","theta","dist","heading_err","v_cmd","omega_cmd"])
                writer.writerows(log)
            print(f"Log saved as sim_log_{mode.replace('+','_')}.csv")

    # Drawing
    screen.fill((30,30,30))
    pygame.draw.circle(screen,(255,0,0),world_to_screen(*TARGET),6)
    if len(trajectory)>1:
        pygame.draw.lines(screen,(0,255,0),False,[world_to_screen(x,y) for (x,y) in trajectory],2)

    # Robot triangle
    theta = robot.theta
    p1 = (robot.x+0.12*math.cos(theta), robot.y+0.12*math.sin(theta))
    p2 = (robot.x-0.07*math.cos(theta)+0.05*math.sin(theta),
          robot.y-0.07*math.sin(theta)-0.05*math.cos(theta))
    p3 = (robot.x-0.07*math.cos(theta)-0.05*math.sin(theta),
          robot.y-0.07*math.sin(theta)+0.05*math.cos(theta))
    pygame.draw.polygon(screen,(200,200,0),[world_to_screen(*p) for p in (p1,p2,p3)])

    info = [
        f"Mode: {mode}",
        f"pos=({robot.x:.2f},{robot.y:.2f})",
        f"theta={robot.theta:.2f}",
        f"stopped={stopped}"
    ]
    for i,t in enumerate(info):
        screen.blit(font.render(t,True,(220,220,220)),(10,10+i*18))

    pygame.display.flip()

pygame.quit()
sys.exit()
