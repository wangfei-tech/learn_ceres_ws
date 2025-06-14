import matplotlib.pyplot as plt

def parse_g2o_with_edges(file_path):
    poses = {}
    landmarks = {}
    edges = []

    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if parts[0] == "TUTORIAL_VERTEX_SE2":
                idx = int(parts[1])
                x, y = float(parts[2]), float(parts[3])
                poses[idx] = (x, y)
            elif parts[0] == "TUTORIAL_VERTEX_POINT_XY":
                idx = int(parts[1])
                x, y = float(parts[2]), float(parts[3])
                landmarks[idx] = (x, y)
            elif parts[0].endswith("TUTORIAL_EDGE_SE2_POINT_XY") or parts[0] == "EDGE_SE2_XY":
                pose_id = int(parts[1])
                landmark_id = int(parts[2])
                edges.append((pose_id, landmark_id))

    return poses, landmarks, edges


def plot_g2o_graph(title, poses, landmarks, edges, color_traj='b', color_landmark='g', color_edge='gray'):
    if poses:
        sorted_keys = sorted(poses.keys())
        traj = [poses[k] for k in sorted_keys]
        x, y = zip(*traj)
        plt.plot(x, y, color=color_traj, linewidth=2, label=f'{title} trajectory')

    if landmarks:
        lx, ly = zip(*landmarks.values())
        plt.scatter(lx, ly, c=color_landmark, marker='o', label=f'{title} landmarks')

    # Draw edges
    for pose_id, lm_id in edges:
        if pose_id in poses and lm_id in landmarks:
            x0, y0 = poses[pose_id]
            x1, y1 = landmarks[lm_id]
            plt.plot([x0, x1], [y0, y1], color=color_edge, linewidth=0.5)

def main():
    before_file = "tutorial_before.g2o"
    after_file = "tutorial_after.g2o"

    poses_b, landmarks_b, edges_b = parse_g2o_with_edges(before_file)
    poses_a, landmarks_a, edges_a = parse_g2o_with_edges(after_file)

    plt.figure(figsize=(12, 10))
    plot_g2o_graph("Before", poses_b, landmarks_b, edges_b, color_traj='red', color_landmark='magenta', color_edge='orange')
    plot_g2o_graph("After", poses_a, landmarks_a, edges_a, color_traj='green', color_landmark='blue', color_edge='cyan')

    plt.legend()
    plt.axis('equal')
    plt.title("G2O Optimization: Pose / Landmarks / Edges")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
