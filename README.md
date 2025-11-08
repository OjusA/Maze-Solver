# 🧭 Maze Solver: Pathfinding Visualization using BFS, Dijkstra’s, and A\*

An interactive Python project that visualizes three major pathfinding algorithms — **Breadth-First Search (BFS)**, **Dijkstra’s Algorithm**, and **A\*** — as they explore and solve a maze in real-time.

Built using **Pygame**, this project demonstrates how intelligent agents can navigate through obstacles to find the shortest path — a key concept in **game development**, **robotics**, and **AI**.

---

## 🎯 **Objective**

To provide a clear, interactive visualization of how popular pathfinding algorithms work in grid-based environments — helping learners and developers understand how shortest path algorithms function and differ in behavior.

---

## ⚙️ **Technologies Used**

- 🐍 **Python 3.12+**
- 🎮 **Pygame**
- 🧠 Algorithms implemented:
  - Breadth-First Search (BFS)
  - Dijkstra’s Algorithm
  - A* (A-Star) Search using Manhattan heuristic

---

## 🚀 **Features**

- 🧱 Interactive maze creation — draw, erase, and design your own maze
- 🟠 Set **Start** and **End** points
- 🔍 Run **BFS**, **Dijkstra**, or **A\*** algorithm with one keypress
- 🎨 Real-time visualization of:
  - **Blue** → OPEN (frontier)
  - **Purple** → CLOSED (visited)
  - **Green** → Shortest path
- 💾 Save and load custom mazes (`S` / `L`)
- ⚙️ Adjustable animation speed (`+` / `-`)
- 🌀 Random maze generator (`G`)
- 🧹 Reset or clear maze easily

---

## 🎮 **Controls**

| Action | Key / Mouse |
|--------|--------------|
| Draw / Remove wall | **Left Click** |
| Set Start & End | **Right Click** |
| Run BFS | **B** |
| Run Dijkstra | **D** |
| Run A\*** | **A** |
| Random Maze | **G** |
| Clear Search | **C** |
| Reset Everything | **R** |
| Save / Load Grid | **S / L** |
| Adjust Speed | **+ / -** |
| Quit | **Esc / Q** |

---

## 🧠 **Algorithm Comparison**

| Algorithm | Uses Heuristic | Weighted | Optimal | Speed | Typical Use |
|------------|----------------|-----------|----------|--------|--------------|
| **BFS** | ❌ No | ❌ No | ✅ Yes | 🐢 Slow | Simple mazes / grids |
| **Dijkstra** | ❌ No | ✅ Yes | ✅ Yes | ⚙️ Moderate | Robotics / GPS |
| **A\*** | ✅ Yes | ✅ Yes | ✅ Yes | 🚀 Fast | Games / Navigation |

---

## 🧩 **Installation and Setup**

### 1️⃣ Clone this repository
```bash
git clone https://github.com/OjusA/Maze-Solver.git
cd Maze-Solver
