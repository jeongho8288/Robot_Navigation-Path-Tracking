import numpy as np
import math
import matplotlib.pyplot as plt

# 파라미터 값 설정
k_pure_pursuit = 0.5  # look forward gain
k = 0.5  # Stanley control gain
Lfc = 2.0  # [m] look-ahead distance for Pure Pursuit
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle
show_animation = True
max_steer = np.radians(30.0)  # [rad] max steering angle

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        delta = np.clip(delta, -max_steer, max_steer)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

def proportional_control(target, current):
    return Kp * (target - current)

class TargetCourse:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):
        if self.old_nearest_point_index is None:
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind
        Lf = k_pure_pursuit * state.v + Lfc  # look forward distance update
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break
            ind += 1
        return ind, Lf

def calculate_cyaw(cx, cy):
    # 경로에 따른 각도 계산
    cyaw = []
    for i in range(1, len(cx)):
        dx = cx[i] - cx[i - 1]
        dy = cy[i] - cy[i - 1]
        cyaw.append(math.atan2(dy, dx))
    cyaw.append(cyaw[-1])  # 마지막 점은 이전 값과 동일하게 설정
    return cyaw

def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)
    if pind >= ind:
        ind = pind
    if ind < len(trajectory.cx):
        tx, ty = trajectory.cx[ind], trajectory.cy[ind]
    else:
        tx, ty = trajectory.cx[-1], trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
    return delta, ind

def stanley_control(state, cx, cy, cyaw, last_target_idx):
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)
    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    theta_d = math.atan2(k * error_front_axle, state.v)
    delta = theta_e + theta_d
    return delta, current_target_idx

def calc_target_index(state, cx, cy):
    fx = state.x + WB * math.cos(state.yaw)
    fy = state.y + WB * math.sin(state.yaw)
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)
    front_axle_vec = [-math.cos(state.yaw + math.pi / 2), -math.sin(state.yaw + math.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
    return target_idx, error_front_axle

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def main():
    # 직사각형 경로 생성
    cx, cy = [], []
    for i in range(100): cx.append(i * 0.5); cy.append(0.0)
    for i in range(60): cx.append(50.0); cy.append(i * 0.5)
    for i in range(100): cx.append(50.0 - i * 0.5); cy.append(30.0)
    for i in range(60): cx.append(0.0); cy.append(30.0 - i * 0.5)

    # 경로에 따른 각도 계산 --> stanley_control에서 사용
    cyaw = calculate_cyaw(cx, cy)

    target_speed = 10.0 / 3.6  # [m/s]
    T = 100.0  # 최대 시뮬레이션 시간 --> 이후 종료

    # Pure Pursuit 알고리즘 초기 state
    pp_state = State(x=0.0, y=-3.0, yaw=0.0, v=0.0)
    # Stanley 초기 상태
    stanley_state = State(x=0.0, y=-3.0, yaw=0.0, v=0.0)

    pp_states, stanley_states = [], []
    pp_target_course = TargetCourse(cx, cy)
    pp_target_ind, _ = pp_target_course.search_target_index(pp_state)
    stanley_target_ind, _ = calc_target_index(stanley_state, cx, cy)

    time = 0.0
    while T >= time and len(cx) - 1 > pp_target_ind and len(cx) - 1 > stanley_target_ind:
        # Pure Pursuit control
        pp_ai = proportional_control(target_speed, pp_state.v)
        pp_di, pp_target_ind = pure_pursuit_steer_control(pp_state, pp_target_course, pp_target_ind)
        pp_state.update(pp_ai, pp_di)
        pp_states.append((pp_state.x, pp_state.y))

        # Stanley control
        stanley_ai = proportional_control(target_speed, stanley_state.v)
        stanley_di, stanley_target_ind = stanley_control(stanley_state, cx, cy, cyaw, stanley_target_ind)
        stanley_state.update(stanley_ai, stanley_di)
        stanley_states.append((stanley_state.x, stanley_state.y))

        time += dt

    # Plot results
    plt.figure()
    plt.plot(cx, cy, "-r", label="course")
    plt.plot([x for x, y in pp_states], [y for x, y in pp_states], "-b", label="Pure Pursuit Trajectory")
    plt.plot([x for x, y in stanley_states], [y for x, y in stanley_states], "-g", label="Stanley Trajectory")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
