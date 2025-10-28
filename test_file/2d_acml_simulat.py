import numpy as np
import matplotlib.pyplot as plt

# 步驟1：建立一個模擬會議室的二值地圖
# 四面牆(1=障礙物)，左側有門口，兩張桌子

def create_meeting_room_map(width=30, height=20):
    grid = np.zeros((height, width), dtype=np.uint8)
    # 四面牆
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1
    # 門口（左牆中間留空）
    door_y = height // 2
    grid[door_y-1:door_y+2, 0] = 0
    # 兩張會議桌（用障礙物方塊表示）
    # 桌1
    grid[4:8, 6:12] = 1
    # 桌2
    grid[12:16, 16:24] = 1
    return grid

def plot_map_with_particles(grid, particles, true_pose=None, title=''):
    plt.imshow(grid, cmap='Greys', origin='lower')
    if particles is not None:
        plt.scatter(particles[:,0], particles[:,1], s=10, c='b', label='Particles', alpha=0.5)
        # 計算粒子群的平均猜測位置
        mean_x = np.mean(particles[:,0])
        mean_y = np.mean(particles[:,1])
        mean_theta = np.arctan2(np.mean(np.sin(particles[:,2])), np.mean(np.cos(particles[:,2])))
        plt.plot(mean_x, mean_y, 'gx', label='Estimated Pose', markersize=12, markeredgewidth=2)
        # 畫出平均朝向
        dx_e = np.cos(mean_theta)
        dy_e = np.sin(mean_theta)
        plt.arrow(mean_x, mean_y, dx_e, dy_e, color='g', head_width=0.5)
    if true_pose is not None:
        plt.plot(true_pose[0], true_pose[1], 'ro', label='True Pose', markersize=10)
        # 畫出朝向
        dx = np.cos(true_pose[2])
        dy = np.sin(true_pose[2])
        plt.arrow(true_pose[0], true_pose[1], dx, dy, color='r', head_width=0.5)
    plt.title(title)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.show()

def random_free_pose(grid):
    h, w = grid.shape
    while True:
        x = np.random.uniform(1, w-1)
        y = np.random.uniform(1, h-1)
        if grid[int(y), int(x)] == 0:
            theta = np.random.uniform(0, 2*np.pi)
            return np.array([x, y, theta])

def simulate_laser_multi(grid, pose, max_range=10, n_beams=8):
    # 多方向雷射，回傳每個方向的距離
    x, y, theta = pose
    angles = theta + np.linspace(0, 2*np.pi, n_beams, endpoint=False)
    dists = []
    for ang in angles:
        for r in np.linspace(0, max_range, 100):
            xi = int(round(x + r * np.cos(ang)))
            yi = int(round(y + r * np.sin(ang)))
            if xi < 0 or xi >= grid.shape[1] or yi < 0 or yi >= grid.shape[0]:
                dists.append(r)
                break
            if grid[yi, xi] == 1:
                dists.append(r)
                break
        else:
            dists.append(max_range)
    return np.array(dists)

def particle_filter_step(grid, particles, true_measure, max_range=10, n_beams=8):
    N = len(particles)
    weights = np.zeros(N)
    for i, p in enumerate(particles):
        pred = simulate_laser_multi(grid, p, max_range, n_beams)
        # 多方向誤差加總
        weights[i] = np.exp(-0.5 * np.sum((pred - true_measure)**2))
    weights += 1e-10
    weights /= np.sum(weights)
    # 重採樣
    indices = np.random.choice(N, N, p=weights)
    new_particles = particles[indices] + np.random.normal(0, 0.2, size=particles.shape)  # 加一點雜訊
    # 保證粒子還在可通行區域
    for i, p in enumerate(new_particles):
        x, y, theta = p
        if x < 1 or x >= grid.shape[1]-1 or y < 1 or y >= grid.shape[0]-1 or grid[int(y), int(x)] == 1:
            new_particles[i] = random_free_pose(grid)
        else:
            new_particles[i,2] = (theta + 2*np.pi) % (2*np.pi)
    return new_particles

if __name__ == '__main__':
    grid = create_meeting_room_map()
    # 初始化真實位置
    true_pose = random_free_pose(grid)
    # 初始化粒子
    N = 200
    particles = np.array([random_free_pose(grid) for _ in range(N)])
    # 顯示初始狀態
    plot_map_with_particles(grid, particles, true_pose, title='Initial Distribution')
    # 多方向雷射參數
    n_beams = 8
    # 模擬多方向雷達量測
    true_measure = simulate_laser_multi(grid, true_pose, n_beams=n_beams)
    # 粒子濾波收斂過程
    for step in range(10):
        particles = particle_filter_step(grid, particles, true_measure, n_beams=n_beams)
        # 計算平均猜測位置
        mean_x = np.mean(particles[:,0])
        mean_y = np.mean(particles[:,1])
        mean_theta = np.arctan2(np.mean(np.sin(particles[:,2])), np.mean(np.cos(particles[:,2])))
        print(f"Step {step+1}: True pose = ({true_pose[0]:.2f}, {true_pose[1]:.2f}, {np.degrees(true_pose[2]):.1f} deg)")
        print(f"           Estimated = ({mean_x:.2f}, {mean_y:.2f}, {np.degrees(mean_theta):.1f} deg)")
        plot_map_with_particles(grid, particles, true_pose, title=f'Particle Filter Step {step+1}')
