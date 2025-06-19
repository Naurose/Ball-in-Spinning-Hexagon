import pygame
import math
import sys

pygame.init()

# Window setup
WIDTH, HEIGHT = 1400, 1000
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Ball in Spinning Hexagon")

clock = pygame.time.Clock()

# Hexagon parameters
hex_radius = 200
hex_center = (WIDTH // 2, HEIGHT // 2)
angle = 0
rotation_speed = 0.02  # faster rotation

# Ball parameters
ball_pos = [WIDTH // 2, HEIGHT // 2 - hex_radius + 40]
ball_radius = 15
ball_velocity = [0, 0]
gravity = 0.5  
bounce_damping = 1.5
friction = 0.98  # simple friction

def get_hexagon_points(center, radius, angle):
    points = []
    for i in range(6):
        theta = angle + i * math.pi / 3
        x = center[0] + radius * math.cos(theta)
        y = center[1] + radius * math.sin(theta)
        points.append((x, y))
    return points

def closest_point_on_segment(px, py, ax, ay, bx, by):
    apx, apy = px - ax, py - ay
    abx, aby = bx - ax, by - ay
    ab_len_sq = abx**2 + aby**2
    dot = apx * abx + apy * aby
    t = max(0, min(1, dot / (ab_len_sq + 1e-10)))
    closest_x = ax + t * abx
    closest_y = ay + t * aby
    return closest_x, closest_y

# Main loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Physics update
    ball_velocity[1] += gravity
    ball_pos[0] += ball_velocity[0]
    ball_pos[1] += ball_velocity[1]

    # Apply friction
    ball_velocity[0] *= friction
    ball_velocity[1] *= friction

    # Rotate hexagon
    angle += rotation_speed
    hexagon_points = get_hexagon_points(hex_center, hex_radius, angle)

    # Collision detection
    min_dist = float('inf')
    nearest_x, nearest_y = ball_pos[0], ball_pos[1]
    nearest_edge_angle = 0

    for i in range(6):
        ax, ay = hexagon_points[i]
        bx, by = hexagon_points[(i + 1) % 6]
        cx, cy = closest_point_on_segment(ball_pos[0], ball_pos[1], ax, ay, bx, by)
        dist = math.hypot(ball_pos[0] - cx, ball_pos[1] - cy)
        if dist < min_dist:
            min_dist = dist
            nearest_x, nearest_y = cx, cy
            # Compute edge angle (for tangential component)
            nearest_edge_angle = math.atan2(by - ay, bx - ax)

    if min_dist < ball_radius:
        # Compute normal
        nx = ball_pos[0] - nearest_x
        ny = ball_pos[1] - nearest_y
        length = math.hypot(nx, ny) + 1e-10
        nx /= length
        ny /= length

        # Correct position
        overlap = ball_radius - min_dist
        ball_pos[0] += nx * overlap
        ball_pos[1] += ny * overlap

        # Reflect velocity
        v_dot_n = ball_velocity[0] * nx + ball_velocity[1] * ny
        ball_velocity[0] -= (1 + bounce_damping) * v_dot_n * nx
        ball_velocity[1] -= (1 + bounce_damping) * v_dot_n * ny

        # Add tangential velocity due to rotating wall
        tangential_speed = rotation_speed * hex_radius
        tx = -math.sin(nearest_edge_angle)
        ty = math.cos(nearest_edge_angle)
        ball_velocity[0] += tx * tangential_speed * 0.05
        ball_velocity[1] += ty * tangential_speed * 0.05

    # Drawing
    screen.fill((30, 30, 30))
    pygame.draw.polygon(screen, (100, 200, 250), hexagon_points, 5)
    pygame.draw.circle(screen, (250, 100, 100), (int(ball_pos[0]), int(ball_pos[1])), ball_radius)

    pygame.display.flip()
    clock.tick(60)
